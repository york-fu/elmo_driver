/** \file
 * \brief elmo motor driver
 *
 */

#include "ec_elmo.h"

#define EC_TIMEOUTMON 500
#define BIT_17 (1 << 17)
#define BIT_19 (1 << 19)
#define BIT_20 (1 << 20)
#define BIT_17_9 (BIT_17 * 9)

uint8 motor_number = 1;

uint32_t rated_current[12] = {0};
uint32_t encoder_range[12] = {
    BIT_17_9, BIT_19, BIT_19, BIT_19, BIT_19, BIT_20,
    BIT_19, BIT_19, BIT_19, BIT_19, BIT_19, BIT_20};

double motor_limit_min[] = {-360, -360, -360, -360, -360, -360, -360, -360, -360, -360, -360, -360};
double motor_limit_max[] = {360, 360, 360, 360, 360, 360, 360, 360, 360, 360, 360, 360};

struct MotorsRead *MotorI[12];
struct MotorsWrite *MotorO[12];

pthread_mutex_t mtx_IOMap;
uint8 _sync_running = 0;
static int _setup_err = 0; // 错误标识

static char IOmap[4096];
static OSAL_THREAD_HANDLE thread_check;
static OSAL_THREAD_HANDLE thread_sync;
static int expectedWKC;
static volatile int wkc;
static boolean inOP;
static uint8 currentgroup = 0;
static int oloop, iloop;

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
          return 0x80; //fault reset
        }
        else
        {
          return 0x06; //shutdown
        }
      }
      else
      {
        return 0x07; //switch on
      }
    }
    else
    {
      return 0x0F; //switch on
    }
  }
  else
  {
    return 0x0F; //switch on
  }

  return 0;
}

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

int sdf_setup(uint16 slave) //六维力设置
{
  int wkc = 0, wc = 0;
  wkc += sdo_write_u8(slave, 0x1a00, 0, 0);
  wc++;
  wkc += sdo_write_u8(slave, 0x1a00, 0, 6); //设置为6输出，默认8输出
  wc++;
  wkc += sdo_write_u8(slave, 0x1C13, 0, 0);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C13, 1, 0x1a00); //pdo设置
  wc++;
  wkc += sdo_write_u16(slave, 0x1C13, 0, 1);
  wc++;
  ec_slave[slave].SM[2].StartAddr = 0;
  ec_slave[slave].SM[4].StartAddr = 0;
  ec_slave[slave].SM[5].StartAddr = 0;
  strncpy(ec_slave[slave].name, "GSV-Left", EC_MAXNAME);
  if (wkc != wc)
  {
    printf("Slave %d GSV-8 setup Failed!!!  wc:%d  wkc:%d\n", slave, wc, wkc);
    _setup_err = 1;
    return -1;
  }
  return 0;
}

int elmo_setup(uint16 slave)
{
  int wkc = 0, wc = 0;
  float64 pmin = -180, pmax = 180;
  uint16 index, check_cnt = 0;
  uint8 subindex, id;
  int32 value_i32 = 0;
  uint32 value_u32 = 0;
  float64 pos;

  if (slave > 0)
    id = slave - 1;
  else
  {
    id = 0;
  }

  pmin = motor_limit_min[id];
  value_i32 = (int32)(pmin * (encoder_range[id] / 360.0));
  sdo_write_i32(slave, 0x607D, 1, value_i32); //Min Software position limit VL[3]
  value_i32 = (int32)(pmin * (encoder_range[id] / 360.0)) - 1;
  sdo_write_i32(slave, 0x607B, 1, value_i32); //Min Position range limit xm[1]

  pmax = motor_limit_max[id];
  value_i32 = (int32)(pmax * (encoder_range[id] / 360.0));
  sdo_write_i32(slave, 0x607D, 2, value_i32); //Max Software position limit VH[3]
  value_i32 = (int32)(pmax * (encoder_range[id] / 360.0)) + 1;
  sdo_write_i32(slave, 0x607B, 2, value_i32); //Max Position range limit xm[2]

  //位置读取
  check_cnt = 0;
  index = 0x6064, subindex = 0; //position actual value
  do
  {
    wkc = 0;
    wkc = sdo_read_i32(slave, index, subindex, &value_i32); //wc++;
    pos = (float64)value_i32 * (360.0 / encoder_range[id]); // 上电角度位置
    check_cnt++;
    if (check_cnt > 100)
    {
      printf("Read position failed of motor %d!\n", id + 1);
      ec_close();
      exit(0);
    }
    osal_usleep(1000);
  } while (wkc == 0);

  if (pos > 360)
  {
    _setup_err = 2;
    printf("Motor %d position > 360\n", id + 1);
  }
  else if (pos < -360)
  {
    _setup_err = 2;
    printf("Motor %d position < -360\n", id + 1);
  }

  // rated current
  check_cnt = 0;
  index = 0x6075, subindex = 0;
  do
  {
    wkc = 0;
    wkc = sdo_read_i32(slave, index, subindex, &value_u32); //wc++;
    rated_current[id] = value_u32;
    check_cnt++;
    if (check_cnt > 100)
    {
      printf("Read rated current failed of motor %d!\n", id + 1);
      ec_close();
      exit(0);
    }
    osal_usleep(1000);
  } while (wkc == 0);
  printf("Motor %d actual position: %3.1f, rated current: %d\n", id + 1, pos, value_u32);

  wkc = 0, wc = 0;
  //PDO Map set by SOD
  //MotorsWrite
  wkc += sdo_write_u8(slave, 0x1C12, 0, 0);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C12, 1, 0x160F);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C12, 2, 0x161C);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C12, 3, 0x160C);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C12, 4, 0x160A);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C12, 5, 0x160B);
  wc++;
  wkc += sdo_write_u16(slave, 0x1C12, 0, 5);
  wc++;

  //MotorsRead
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
    printf("Slave %d setup Failed!!!  wc:%d  wkc:%d \n", slave, wc, wkc);
    _setup_err = 3;
    return -1;
  }
  return 0;
}

void set_config_hook()
{
  int slave_idx;
  for (slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
  {
    ec_slavet *slave = &ec_slave[slave_idx];
    if (slave->eep_man == 0x5 && slave->eep_id == 0x80000) //SDF１ set
    {
      slave->PO2SOconfig = sdf_setup;
    }
    if (slave->eep_man == 0x270 && slave->eep_id == 0x80000) //SDF２ set
    {
      slave->PO2SOconfig = sdf_setup;
    }
    if (slave->eep_man == 0x9a && slave->eep_id == 0x30924) //ELMO set
    {
      slave->PO2SOconfig = elmo_setup;
    }
  }
}

int8_t set_ec_state(ec_state sta)
{
  int chk;
  ec_slave[0].state = sta;
  /* send one valid process data to make outputs in slaves happy*/
  pthread_mutex_lock(&mtx_IOMap);
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  pthread_mutex_unlock(&mtx_IOMap);
  /* request sta state for all slaves */
  ec_writestate(0);
  chk = 10;
  /* wait for all slaves to reach sta state */
  do
  {
    pthread_mutex_lock(&mtx_IOMap);
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, sta, 50000);
    pthread_mutex_unlock(&mtx_IOMap);
  } while (chk-- && (ec_slave[0].state != sta));

  if (ec_slave[0].state == sta)
  {
    printf("Set state reached for all slaves.\n");
    return 0;
  }
  return 1;
}

int16_t elmo_config(char *ifname)
{
  int chk;
  /* initialise SOEM, bind socket to ifname */
  if (ec_init(ifname) == 0)
  {
    printf("No socket connection on %s\nExcecute as root\n", ifname);
    return 1;
  }
  printf("ec_init on %s succeeded.\n", ifname);

  if (ec_config_init(FALSE) <= 0)
  {
    printf("No slaves found!\n");
    ec_close();
    return 2;
  }
  printf("%d slaves found and configured.\n", ec_slavecount);

  set_config_hook();
  ec_config_map(&IOmap);
  ec_configdc();

  if (_setup_err != 0)
    return 3;

  oloop = ec_slave[0].Obytes;
  if ((oloop == 0) && (ec_slave[0].Obits > 0))
    oloop = 1;
  iloop = ec_slave[0].Ibytes;
  if ((iloop == 0) && (ec_slave[0].Ibits > 0))
    iloop = 1;
  expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  printf("oloop:%d  iloop:%d  expectedWKC:%d\n", oloop, iloop, expectedWKC);

  /* wait for all slaves to reach SAFE_OP state */
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

  printf("Request operational state for all slaves\n");
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  /* send one valid process data to make outputs in slaves happy*/
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  /* request OP state for all slaves */
  ec_writestate(0);
  chk = 40;
  /* wait for all slaves to reach OP state */
  do
  {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
  } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  if (ec_slave[0].state == EC_STATE_OPERATIONAL)
  {
    printf("Operational state reached for all slaves.\n");
    inOP = TRUE;
    return 0;
  }
  ec_slave[0].state = EC_STATE_INIT;
  /* request INIT state for all slaves */
  ec_writestate(0);
  return 4;
}

OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
  int slave;
  (void)ptr; /* Not used */

  while (_sync_running)
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

int16 motor_is_error()
{
  uint16 SWord;
  uint16 index, value_u16;
  uint8 subindex, value_u8;
  for (int i = 0; i < motor_number; i++)
  {
    SWord = MotorI[i]->status_word;
    SWord = SWord & 0x6f;
    if (SWord != 0x27)
    {
      printf("Motor %d state error, status_word:0x%x\n", i + 1, MotorI[i]->status_word);
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
  (void)ptr;
  double cycle_time = 0.001; // s
  struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (_sync_running)
  {
    pthread_mutex_lock(&mtx_IOMap);
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    if (motor_is_error())
    {
      pthread_mutex_unlock(&mtx_IOMap);
      set_ec_state(EC_STATE_PRE_OP);
      ec_close();
      exit(EXIT_FAILURE);
    }
    pthread_mutex_unlock(&mtx_IOMap);

    next_time.tv_sec += (next_time.tv_nsec + cycle_time * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + cycle_time * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

int8 structural_map()
{
  int rl = sizeof(MotorsRead);
  int wl = sizeof(MotorsWrite);
  if (oloop == (wl * motor_number) && iloop == (rl * motor_number)) // 结构体映射
  {
    for (uint32_t i = 0; i < motor_number; i++)
    {
      MotorI[i] = (struct MotorsRead *)(ec_slave[0].inputs + rl * i);
      MotorO[i] = (struct MotorsWrite *)(ec_slave[0].outputs + wl * i);
    }
    return 0;
  }
  return 1;
}

int8 single_motor_enable(uint16 id)
{
  if (id < 1)
  {
    printf("Illegal input id %d!", id);
    return 1;
  }
  uint16 SWord;
  MotorO[id - 1]->mode_of_opration = MODE_CSP;
  MotorO[id - 1]->target_position = MotorI[id - 1]->position_actual_value;
  MotorO[id - 1]->target_velocity = 0;
  MotorO[id - 1]->target_torque = 0;
  SWord = MotorI[id - 1]->status_word & 0x6f;
  MotorO[id - 1]->control_word = ctrlWord(SWord);
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

int8 motor_enable(void)
{
  int8 result = 0;
  printf("Wait %d all motor enable...\n", motor_number);
  osal_usleep(1000);
  for (int j = 1; j <= motor_number; j++)
  {
    for (int c = 0; c < 5000; c++)
    {
      result = single_motor_enable(j);
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

int8_t ec_elmo_init(char *ifname)
{
  int16_t ret = 0;
  _sync_running = 1;
  osal_thread_create(&thread_check, 128000, &ecatcheck, (void *)&ctime);
  ret = elmo_config(ifname);
  if (ret != 0)
  {
    return 1;
  }
  ret = structural_map();
  if (ret != 0)
  {
    printf("structural map failed, return %d\n", ret);
    return 2;
  }
  ret = motor_enable();
  if (ret != 0)
  {
    return 3;
  }
  pthread_mutex_init(&mtx_IOMap, NULL);
  osal_thread_create_rt(&thread_sync, 204800, &sync_thread, NULL);
  if (ret != 0)
  {
    printf("motor enable failed, return %d\n", ret);
    return 4;
  }
  return 0;
}
