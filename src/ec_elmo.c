#include "ec_elmo.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread_ecatcheck;
OSAL_THREAD_HANDLE thread_sync;
int expectedWKC;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
int oloop, iloop;

static MotorConfig_t *motor_cfg;
static uint8_t running = 0;
static pthread_mutex_t mtx;
struct ELMORead *elmoI[NUM_SLAVE_MAX];
struct ELMOWrite *elmoO[NUM_SLAVE_MAX];

int elmo_PDOconfig(uint16 slave)
{
  uint8 u8val;
  uint16 u16val;
  int wkc = 0, wkc_ref = 0;

  u8val = 0;
  wkc += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
  wkc_ref++;
  u16val = 0x1605;
  wkc += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
  wkc_ref++;
  u16val = 0x1616;
  wkc += ec_SDOwrite(slave, 0x1c12, 0x02, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
  wkc_ref++;
  u16val = 0x1617;
  wkc += ec_SDOwrite(slave, 0x1c12, 0x03, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
  wkc_ref++;
  u16val = 0x1618;
  wkc += ec_SDOwrite(slave, 0x1c12, 0x04, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
  wkc_ref++;
  u8val = 4;
  wkc += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
  wkc_ref++;

  u8val = 0;
  wkc += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
  wkc_ref++;
  u16val = 0x1a03;
  wkc += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
  wkc_ref++;
  u16val = 0x1a13;
  wkc += ec_SDOwrite(slave, 0x1c13, 0x02, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
  wkc_ref++;
  u16val = 0x1a0b;
  wkc += ec_SDOwrite(slave, 0x1c13, 0x03, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
  wkc_ref++;
  u8val = 3;
  wkc += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
  wkc_ref++;

  ec_slave[slave].SM[4].StartAddr = 0;
  ec_slave[slave].SM[5].StartAddr = 0;

  if (wkc != wkc_ref)
  {
    printf("Failed to PDO config, at %d, wkc: %d, wkc ref: %d\n", slave, wkc, wkc_ref);
    ec_close();
    exit(0);
  }

  int rdl;
  int32 i32val = 0;
  uint32 u32val = 0;
  uint16 chk;
  wkc = 0;

  chk = 100;
  while (!wkc && (chk--))
  {
    rdl = sizeof(i32val), i32val = 0;
    wkc = ec_SDOread(slave, 0x6075, 0x00, FALSE, &rdl, &i32val, EC_TIMEOUTRXM);
    osal_usleep(1000);
  }
  if (chk != 0)
  {
    motor_cfg->rated_current[slave - 1] = i32val;
  }
  else
  {
    printf("Failedto read rated current, at %d\n", slave);
    ec_close();
    exit(0);
  }

  chk = 100;
  while (!wkc && (chk--))
  {
    rdl = sizeof(i32val), i32val = 0;
    wkc = ec_SDOread(slave, 0x6064, 0x00, FALSE, &rdl, &i32val, EC_TIMEOUTRXM);
    osal_usleep(1000);
  }
  if (chk != 0)
  {
    printf("Initial position %f, at %d\n", (double)i32val / (motor_cfg->range[slave - 1] * motor_cfg->gear) * motor_cfg->circle_unit, slave);
  }
  else
  {
    printf("Failedto read position, at %d\n", slave);
  }

  motor_cfg->num++;

  return 0;
}

void setup_hook()
{
  for (uint16_t i = 1; i <= ec_slavecount; i++)
  {
    if ((ec_slave[i].eep_man == 0x9a) && (ec_slave[i].eep_id == 0x30924))
    {
      ec_slave[i].PO2SOconfig = &elmo_PDOconfig;
    }
  }
}

int8 set_ec_state(ec_state state)
{
  int chk;
  ec_slave[0].state = state;
  /* request OP state for all slaves */
  ec_writestate(0);
  pthread_mutex_lock(&mtx);
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  pthread_mutex_unlock(&mtx);
  chk = 20;
  /* wait for all slaves to reach OP state */
  do
  {
    pthread_mutex_lock(&mtx);
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    pthread_mutex_unlock(&mtx);
    ec_statecheck(0, state, 50000);
  } while (chk-- && (ec_slave[0].state != state));

  if (ec_slave[0].state == state)
  {
    return 0;
  }
  return 1;
}

pthread_mutex_t *ec_elmo_get_mtx()
{
  return &mtx;
}

int8_t ec_elmo_get_data_ptr(ELMORead_t **in, ELMOWrite_t **out)
{
  in = elmoI;
  out = elmoO;
  return 0;
}

int8_t sw_check()
{
  uint16_t index, sw;
  for (uint8_t i = 0; i < motor_cfg->num; i++)
  {
    sw = elmoI[i]->status_word & 0x6f;
    if (sw != 0x27)
    {
      printf("Motor %d state error, status word: 0x%x\n", i + 1, elmoI[i]->status_word);

      int wkc = 0;
      uint16_t idx = 0;
      uint8_t sub_idx = 0;
      uint8_t u8val = 0;
      uint16_t u16val = 0;
      idx = 0x1001, sub_idx = 0x00, wkc = 0;
      wkc = ec_SDOread(i, idx, sub_idx, FALSE, (int *)(sizeof(u8val)), &u8val, EC_TIMEOUTRXM);
      printf("0x%x:%d = 0x%x, wkc = %d\n", idx, sub_idx, u8val, wkc);
      idx = 0x603f, sub_idx = 0x00, wkc = 0;
      wkc = ec_SDOread(i, idx, sub_idx, FALSE, (int *)(sizeof(u16val)), &u16val, EC_TIMEOUTRXM);
      printf("0x%x:%d = 0x%x, wkc = %d\n", idx, sub_idx, u16val, wkc);

      return -1;
    }
  }
  return 0;
}

int8 structural_map()
{
  int rl = sizeof(struct ELMORead);
  int wl = sizeof(struct ELMOWrite);
  if (oloop == (wl * motor_cfg->num) && iloop == (rl * motor_cfg->num))
  {
    for (uint32_t i = 0; i < motor_cfg->num; i++)
    {
      elmoI[i] = (struct ELMORead *)(ec_slave[0].inputs + rl * i);
      elmoO[i] = (struct ELMOWrite *)(ec_slave[0].outputs + wl * i);
    }
    return 0;
  }
  printf("Failed to structural map\n");
  return -1;
}

OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
  int slave;
  (void)ptr; /* Not used */

  while (running)
  {
    if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
    {
      /* one ore more slaves are not responding */
      ec_group[currentgroup].docheckstate = FALSE;
      ec_readstate();
      for (slave = 1; slave <= ec_slavecount; slave++)
      {
        if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
        {
          ec_group[currentgroup].docheckstate = TRUE;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
          {
            printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
          }
          else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
          {
            printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
          }
          else if (ec_slave[slave].state > EC_STATE_NONE)
          {
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
            {
              ec_slave[slave].islost = FALSE;
              printf("MESSAGE : slave %d reconfigured\n", slave);
            }
          }
          else if (!ec_slave[slave].islost)
          {
            /* re-check state */
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (ec_slave[slave].state == EC_STATE_NONE)
            {
              ec_slave[slave].islost = TRUE;
              printf("ERROR : slave %d lost\n", slave);
            }
          }
        }
        if (ec_slave[slave].islost)
        {
          if (ec_slave[slave].state == EC_STATE_NONE)
          {
            if (ec_recover_slave(slave, EC_TIMEOUTMON))
            {
              ec_slave[slave].islost = FALSE;
              printf("MESSAGE : slave %d recovered\n", slave);
            }
          }
          else
          {
            ec_slave[slave].islost = FALSE;
            printf("MESSAGE : slave %d found\n", slave);
          }
        }
      }
      if (!ec_group[currentgroup].docheckstate)
        printf("OK : all slaves resumed OPERATIONAL.\n");
    }
    osal_usleep(10000);
  }
}

OSAL_THREAD_FUNC sync_thread(void *ptr)
{
  double dt = *(double *)ptr;
  printf("Ethercat synchronization period: %f\n", dt);
  struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (running)
  {
    pthread_mutex_lock(&mtx);
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    pthread_mutex_unlock(&mtx);

    if (motor_cfg->sw_check)
    {
      if (sw_check() != 0)
      {
        set_ec_state(EC_STATE_PRE_OP);
        ec_close();
        exit(EXIT_FAILURE);
      }
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

int8_t ec_elmo_init(ECMConfig_t ec_cfg, MotorConfig_t *m_cfg)
{
  /* initialise SOEM, bind socket to ifname */
  if (!ec_init(ec_cfg.ifname))
  {
    printf("No socket connection on %s\nExecute as root\n", ec_cfg.ifname);
    return -1;
  }
  printf("ec_init on %s succeeded.\n", ec_cfg.ifname);

  /* find and auto-config slaves */
  if (ec_config_init(FALSE) < 1)
  {
    ec_close();
    printf("No slaves found!\n");
    return -2;
  }
  printf("%d slaves found and configured.\n", ec_slavecount);

  motor_cfg = m_cfg;
  motor_cfg->num = 0;
  setup_hook();
  ec_config_map(&IOmap);
  ec_configdc();

  printf("Slaves mapped, state to SAFE_OP.\n");
  /* wait for all slaves to reach SAFE_OP state */
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

  oloop = ec_slave[0].Obytes;
  if ((oloop == 0) && (ec_slave[0].Obits > 0))
    oloop = 1;
  iloop = ec_slave[0].Ibytes;
  if ((iloop == 0) && (ec_slave[0].Ibits > 0))
    iloop = 1;
  expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  printf("oloop:%d  iloop:%d  expectedWKC:%d\n", oloop, iloop, expectedWKC);

  printf("Request operational state for all slaves\n");
  if (set_ec_state(EC_STATE_OPERATIONAL) != 0)
  {
    return -4;
  }
  printf("Operational state reached for all slaves.\n");
  inOP = TRUE;

  running = 1;

  int ret = 0;
  ret = osal_thread_create(&thread_ecatcheck, 128000, &ecatcheck, (void *)&ctime);
  if (ret != 1)
  {
    printf("Failed to create 'ecatcheck' thread, return %d\n", ret);
    return -3;
  }

  ret = osal_thread_create_rt(&thread_sync, 204800, &sync_thread, (void *)(&ec_cfg.dt));
  if (ret != 1)
  {
    printf("Failed to create 'sync_thread' thread, return %d\n", ret);
    return -3;
  }
  osal_usleep(1000);

  structural_map();

  return 0;
}

int8_t ec_elmo_deinit()
{
  running = 0;
  osal_usleep(1000 * 5);
  set_ec_state(EC_STATE_INIT);
  ec_close();
  return 0;
}