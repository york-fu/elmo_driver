
#include "ec_elmo.h"

#define EC_TIMEOUTMON 500

#define STATUSWORD_OPERATION_ENABLE_BIT 2
#define STATUSWORD_SWITCHED_ON_BIT 1
#define STATUSWORD_READY_TO_SWITCH_ON_BIT 0
#define STATUSWORD_FAULT_BIT 3

char IOmap[4096];
OSAL_THREAD_HANDLE thread_ecatcheck;
int expectedWKC;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
int oloop, iloop;

static uint8 position_limit = 0;
static double position_limit_min[NUM_SLAVE_MAX] = {0};
static double position_limit_max[NUM_SLAVE_MAX] = {0};
static uint16 num_motor = 0;
static uint8 running = 0;

uint32 _encoder_range[NUM_SLAVE_MAX] = {0};
uint32 _rated_current[NUM_SLAVE_MAX] = {0};

struct ELMORead *elmoI[NUM_SLAVE_MAX];
struct ELMOWrite *elmoO[NUM_SLAVE_MAX];

pthread_mutex_t mtx;
OSAL_THREAD_HANDLE thread_sync;

int sdo_write_u8(uint16 slave, uint16 index, uint8 subindex, uint8 value)
{
  int wkc;
  wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
  return wkc;
}

int sdo_write_u16(uint16 slave, uint16 index, uint8 subindex, uint16 value)
{
  int wkc;
  wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
  return wkc;
}

int sdo_write_u32(uint16 slave, uint16 index, uint8 subindex, uint32 value)
{
  int wkc;
  wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
  return wkc;
}

int sdo_write_i32(uint16 slave, uint16 index, uint8 subindex, int32 value)
{
  int wkc;
  wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
  return wkc;
}

int sdo_read_u8(uint16 slave, uint16 index, uint8 subindex, uint8 *value)
{
  int wkc;
  int l = sizeof(value);
  wkc = ec_SDOread(slave, index, subindex, FALSE, &l, value, EC_TIMEOUTRXM);
  return wkc;
}

int sdo_read_u16(uint16 slave, uint16 index, uint8 subindex, uint16 *value)
{
  int wkc;
  int l = sizeof(value);
  wkc = ec_SDOread(slave, index, subindex, FALSE, &l, value, EC_TIMEOUTRXM);
  return wkc;
}

int sdo_read_u32(uint16 slave, uint16 index, uint8 subindex, uint32 *value)
{
  int wkc;
  int l = sizeof(value);
  wkc = ec_SDOread(slave, index, subindex, FALSE, &l, value, EC_TIMEOUTRXM);
  return wkc;
}

int sdo_read_i32(uint16 slave, uint16 index, uint8 subindex, int32 *value)
{
  int wkc;
  int l = sizeof(value);
  wkc = ec_SDOread(slave, index, subindex, FALSE, &l, value, EC_TIMEOUTRXM);
  return wkc;
}

int elmo_motor_config(uint16 slave)
{
  int wkc = 0, wc = 0;
  uint8 id;
  uint16 index;
  uint8 subindex;
  int32 value_i32 = 0;
  uint32 value_u32 = 0;
  uint16 chk;
  float64 initial_pos;

  if (slave > 0)
    id = slave - 1;
  else
  {
    id = 0;
  }

  wkc = 0;
  chk = 0;
  index = 0x6064, subindex = 0; // position actual value
  do
  {
    wkc = sdo_read_i32(slave, index, subindex, &value_i32);
    chk++;
    if (chk > 100)
    {
      printf("Read position failed of motor %d!\n", id + 1);
      ec_close();
      exit(0);
    }
    osal_usleep(1000);
  } while (wkc == 0);
  initial_pos = (float64)value_i32 * (360.0 / _encoder_range[id]);

  wkc = 0;
  chk = 0;
  index = 0x6075, subindex = 0; // rated current
  do
  {
    wkc = sdo_read_i32(slave, index, subindex, &value_u32);
    chk++;
    if (chk > 100)
    {
      printf("Read rated current failed of motor %d!\n", id + 1);
      ec_close();
      exit(0);
    }
    osal_usleep(1000);
  } while (wkc == 0);
  _rated_current[id] = value_u32;
  printf("Motor %d actual position: %.2f, rated current: %.3f\n", id + 1, initial_pos, value_u32 / 1000.0);

  if (position_limit > 0)
  {
    float64 p_min = -180, p_max = 180;
    wkc = 0, wc = 0;

    p_min = position_limit_min[id];
    value_i32 = (int32)(p_min * (_encoder_range[id] / 360.0));
    wkc += sdo_write_i32(slave, 0x607D, 1, value_i32); // Min Software position limit VL[3]
    wc++;
    value_i32 = (int32)(p_min * (_encoder_range[id] / 360.0)) - 1;
    wkc += sdo_write_i32(slave, 0x607B, 1, value_i32); // Min Position range limit xm[1]
    wc++;

    p_max = position_limit_max[id];
    value_i32 = (int32)(p_max * (_encoder_range[id] / 360.0));
    wkc += sdo_write_i32(slave, 0x607D, 2, value_i32); // Max Software position limit VH[3]
    wc++;
    value_i32 = (int32)(p_max * (_encoder_range[id] / 360.0)) + 1;
    wkc += sdo_write_i32(slave, 0x607B, 2, value_i32); // Max Position range limit xm[2]
    wc++;

    if (wkc != wc)
    {
      printf("Slave %d set software position limit failed! wc:%d, wkc:%d", slave, wc, wkc);
      ec_close();
      exit(0);
    }
  }

  wkc = 0, wc = 0;
  // PDO Map set by SOD
  // ELMOWrite
  wkc += sdo_write_u8(slave, 0x1C12, 0, 0);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C12, 1, 0x1605);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C12, 2, 0x1616);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C12, 3, 0x1617);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C12, 4, 0x1618);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C12, 0, 4);
  wc++;
  // ELMORead
  wkc += sdo_write_u8(slave, 0x1C13, 0, 0);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C13, 1, 0x1A03);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C13, 2, 0x1A13);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C13, 3, 0x1A0B);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C13, 0, 3);
  wc++;
  ec_slave[slave].SM[4].StartAddr = 0;
  ec_slave[slave].SM[5].StartAddr = 0;
  if (wkc != wc)
  {
    printf("Slave %d PDO map failed! wc:%d, wkc:%d", slave, wc, wkc);
    ec_close();
    exit(0);
  }
  return 0;
}

int elmo_sdf_config(uint16 slave) //六维力设置
{
  int wkc = 0, wc = 0;
  wkc += sdo_write_u8(slave, 0x1a00, 0, 0);
  wc++;
  wkc += sdo_write_u8(slave, 0x1a00, 0, 6); //设置为6输出，默认8输出
  wc++;
  wkc += sdo_write_u8(slave, 0x1C13, 0, 0);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C13, 1, 0x1a00); // pdo设置
  wc++;
  wkc += sdo_write_u16(slave, 0x1C13, 0, 1);
  wc++;
  ec_slave[slave].SM[2].StartAddr = 0;
  ec_slave[slave].SM[4].StartAddr = 0;
  ec_slave[slave].SM[5].StartAddr = 0;
  strncpy(ec_slave[slave].name, "GSV-Left", EC_MAXNAME);
  if (wkc != wc)
  {
    printf("Slave %d GSV-8 setup failed! wc:%d, wkc:%d", slave, wc, wkc);
    ec_close();
    exit(0);
  }
  return 0;
}

int8 elmo_hook_config()
{
  int slave_idx;
  for (slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
  {
    ec_slavet *slave = &ec_slave[slave_idx];
    if (slave->eep_man == 0x9a && slave->eep_id == 0x30924) // ELMO set
    {
      slave->PO2SOconfig = elmo_motor_config;
      num_motor++;
    }
    if (slave->eep_man == 0x5 && slave->eep_id == 0x80000) // SDF１ set
    {
      slave->PO2SOconfig = elmo_sdf_config;
    }
    if (slave->eep_man == 0x270 && slave->eep_id == 0x80000) // SDF２ set
    {
      slave->PO2SOconfig = elmo_sdf_config;
    }
  }
}

uint16 ctrlWord(uint16 state_word) //状态字到控制字
{
  if (!(state_word & (1 << STATUSWORD_OPERATION_ENABLE_BIT)))
  {
    if (!(state_word & (1 << STATUSWORD_SWITCHED_ON_BIT)))
    {
      if (!(state_word & (1 << STATUSWORD_READY_TO_SWITCH_ON_BIT)))
      {
        if ((state_word & (1 << STATUSWORD_FAULT_BIT)))
        {
          return 0x80; // fault reset
        }
        else
        {
          return 0x06; // shutdown
        }
      }
      else
      {
        return 0x07; // switch on
      }
    }
    else
    {
      return 0x0F; // switch on
    }
  }
  else
  {
    return 0x0F; // switch on
  }

  return 0;
}

int8 structural_map()
{
  int rl = sizeof(ELMORead);
  int wl = sizeof(ELMOWrite);
  if (oloop == (wl * num_motor) && iloop == (rl * num_motor))
  {
    for (uint32_t i = 0; i < num_motor; i++)
    {
      elmoI[i] = (struct ELMORead *)(ec_slave[0].inputs + rl * i);
      elmoO[i] = (struct ELMOWrite *)(ec_slave[0].outputs + wl * i);
    }
    return 0;
  }
  printf("Failed to structural map!");
  return -1;
}

int8 elmo_motor_enable(uint16 id)
{
  if (id < 1)
  {
    printf("Illegal input! id:%d\n", id);
    return -1;
  }
  uint16 SWord;
  uint16 index = id - 1;
  elmoO[index]->target_position = elmoI[index]->position_actual_value;
  elmoO[index]->position_offset = 0;
  elmoO[index]->velocit_offset = 0;
  elmoO[index]->torque_offset = 0;
  elmoO[index]->max_torque = 1000;
  elmoO[index]->mode_of_opration = MODE_CSP;
  SWord = elmoI[index]->status_word & 0x6f;
  elmoO[index]->control_word = ctrlWord(SWord);
  ec_send_processdata();
  wkc = ec_receive_processdata(EC_TIMEOUTRET);
  if (SWord == 0x27)
  {
    printf("Motor %d enable success!\n", id);
    return 0;
  }
  else
    return 2;
}

int8 elmo_all_motor_enable(void)
{
  int8 result = 0;
  printf("Wait %d all motor enable...\n", num_motor);
  osal_usleep(1000);
  for (int j = 1; j <= num_motor; j++)
  {
    for (int c = 0; c < 5000; c++)
    {
      result = elmo_motor_enable(j);
      if (result == 0)
        break;
      osal_usleep(1000);
    }
    if (result != 0)
    {
      printf("Wait motor %d enable failed!\n", j);
      return 1;
    }
  }
  return 0;
}

int8 elmo_set_state(ec_state state)
{
  int chk;
  ec_slave[0].state = state;
  /* send one valid process data to make outputs in slaves happy*/
  pthread_mutex_lock(&mtx);
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  pthread_mutex_unlock(&mtx);
  /* request OP state for all slaves */
  ec_writestate(0);
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

int16 elmo_is_error()
{
  uint16 SWord;
  uint16 index, value_u16;
  uint8 subindex, value_u8;
  for (int i = 0; i < num_motor; i++)
  {
    SWord = elmoI[i]->status_word & 0x6f;
    if (SWord != 0x27)
    {
      printf("Elmo %d state error, status_word:0x%x\n", i + 1, elmoI[i]->status_word);
      index = 0x1001, subindex = 0;
      sdo_read_u8(i + 1, index, subindex, &value_u8);
      printf("0x%x:%d=0x%x\n", index, subindex, value_u8);
      index = 0x603F, subindex = 0;
      sdo_read_u16(i + 1, index, subindex, &value_u16);
      printf("0x%x:%d=0x%x\n", index, subindex, value_u16);
      return i + 1;
    }
  }
  return 0;
}

OSAL_THREAD_FUNC_RT sync_thread(void *ptr)
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
    if (elmo_is_error())
    {
      elmo_set_state(EC_STATE_PRE_OP);
      ec_close();
      exit(EXIT_FAILURE);
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

int8 elmo_board_init()
{
  elmo_hook_config();
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
  if (elmo_set_state(EC_STATE_OPERATIONAL) != 0)
  {
    return -1;
  }
  printf("Operational state reached for all slaves.\n");
  inOP = TRUE;

  return 0;
}

int8 elmo_motor_init()
{
  if (structural_map() != 0)
  {
    return -1;
  }
  if (elmo_all_motor_enable() != 0)
  {
    return -2;
  }
  return 0;
}

int8 elmo_init(const char *ifname, double dt, MotorOptions_t opt)
{
  /* initialise SOEM, bind socket to ifname */
  if (ec_init(ifname) == 0)
  {
    printf("No socket connection on %s\nExecute as root\n", ifname);
    return -1;
  }
  printf("ec_init on %s succeeded.\n", ifname);
  if (ec_config_init(FALSE) < 1)
  {
    printf("No slaves found!\n");
    /* stop SOEM, close socket */
    ec_close();
    return -2;
  }
  printf("%d slaves found and configured.\n", ec_slavecount);

  running = 1;
  int ret;
  ret = osal_thread_create(&thread_ecatcheck, 128000, &ecatcheck, (void *)&ctime);
  if (ret != 1)
  {
    printf("Failed to create 'ecatcheck' thread, return %d\n", ret);
    return -3;
  }

  if (opt.encoder_range == NULL)
  {
    printf("No set encoder range!\n");
    return -3;
  }
  for (uint16 i = 0; i < opt.size; i++)
  {
    _encoder_range[i] = opt.encoder_range[i];
  }
  if (opt.position_limit != 0)
  {
    position_limit = opt.position_limit;
    for (uint16 i = 0; i < opt.size; i++)
    {
      position_limit_min[i] = opt.position_limit_min[i];
      position_limit_max[i] = opt.position_limit_max[i];
    }
  }
  if (elmo_board_init() != 0)
  {
    return -4;
  }

  if (elmo_motor_init() != 0)
  {
    return -5;
  }

  double dt_ = dt;
  ret = osal_thread_create_rt(&thread_sync, 204800, &sync_thread, (void *)(&dt));
  if (ret != 1)
  {
    printf("Failed to create 'sync_thread' thread, return %d\n", ret);
    return -6;
  }
  return 0;
}

int8 elmo_deinit()
{
  running = 0;
  osal_usleep(1000 * 5);
  elmo_set_state(EC_STATE_INIT);
  ec_close();
  return 0;
}
