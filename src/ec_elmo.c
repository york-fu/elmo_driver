/** \file
 * \brief elmo motor driver
 *
 */

#include "ec_elmo.h"

#define EC_TIMEOUTMON 500
#define BIT_19 (1 << 19)
#define BIT_20 (1 << 20)

char IOmap[4096];
OSAL_THREAD_HANDLE thread_check;
OSAL_THREAD_HANDLE thread_sync;
int expectedWKC;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
int oloop, iloop;
int error_num = 0; //全局错误标识
uint8 joint_num = 1;

uint32_t encoder_range[12] = {
    BIT_20, BIT_19, BIT_19, BIT_19, BIT_19, BIT_20,
    BIT_19, BIT_19, BIT_19, BIT_19, BIT_19, BIT_20};

double JonitLimitmax[] = {90, 90, 120, 65, 75, 220, 45, 60, 120, 120, 105, 110};
double JonitLimitmin[] = {-90, 30, -60, -60, -40, 0, 15, 0, -60, 25, -5, -110};

struct JointsRead *JointI[12];
struct JointsWrite *JointO[12];

uint16 to_ctrl_word(uint16 state_word) //状态字到控制字
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
    error_num = 1;
    return -1;
  }
  return 0;
}

int elmo_setup(uint16 slave)
{
  int wkc = 0, wc = 0;
  float64 pmin = -180, pmax = 180;
  uint16 index, check_cnt = 0;
  uint8 subindex, id; //0不显示设置信息
  int32 value_i32 = 0;
  float64 pos;

  if (slave > 1)
  {
    if (slave < 9)
      id = slave - 2;
    else
      id = slave - 3;
  }
  else
    id = 0;

  //Non-modulo motion.
  pmin = JonitLimitmin[id];
  value_i32 = (int32)(pmin * (encoder_range[id] / 360.0));
  sdo_write_i32(slave, 0x607D, 1, value_i32); //Min Software position limit VL[3]
  value_i32 = (int32)(pmin * (encoder_range[id] / 360.0)) - 1;
  sdo_write_i32(slave, 0x607B, 1, value_i32); //Min Position range limit xm[1]

  pmax = JonitLimitmax[id];
  value_i32 = (int32)(pmax * (encoder_range[id] / 360.0));
  sdo_write_i32(slave, 0x607D, 2, value_i32); //Max Software position limit VH[3]
  value_i32 = (int32)(pmax * (encoder_range[id] / 360.0)) + 1;
  sdo_write_i32(slave, 0x607B, 2, value_i32); //Max Position range limit xm[2]

  //位置读取
  index = 0x6064, subindex = 0; //position actual value
  do
  {
    wkc = 0;
    wkc = sdo_read_i32(slave, index, subindex, &value_i32); //wc++;
    pos = (float64)value_i32 * (360.0 / encoder_range[id]);    // 上电角度位置
    check_cnt++;
    if (check_cnt > 100)
    {
      printf("Read position failed of joint%d!\n", id + 1);
      exit(0);
    }
    osal_usleep(1000);
  } while (wkc == 0);
  printf("Joint%d\t  Start_Angle: %3.1f\t ", id + 1, pos);

  if (pos > 180)
  {
    error_num = 3;
    printf(" >180\n");
  }
  else if (pos < -180)
  {
    error_num = 3;
    printf(" <-180\n");
  }
  else
  {
    printf(" normal\n");
  }
  //SDO set from TWINCAT
  sdo_write_u16(slave, 0x605e, 0, 0);
  sdo_write_u16(slave, 0x6060, 0, 8);
  sdo_write_u32(slave, 0x6065, 0, 436970);
  sdo_write_u16(slave, 0x6066, 0, 20);
  sdo_write_i32(slave, 0x6067, 0, 100);
  sdo_write_u16(slave, 0x6068, 0, 1);
  sdo_write_u16(slave, 0x606e, 0, 1);
  sdo_write_u16(slave, 0x6070, 0, 1);
  sdo_write_u32(slave, 0x6075, 0, 28280);
  sdo_write_u32(slave, 0x6076, 0, 28280);
  sdo_write_u32(slave, 0x607f, 0, 5242880);
  sdo_write_u32(slave, 0x6087, 0, 10000000);
  sdo_write_u8(slave, 0x60C2, 1, 2);
  sdo_write_u32(slave, 0x60c5, 0, 72900);
  sdo_write_u32(slave, 0x60c6, 0, 72900);
  sdo_write_u32(slave, 0x60fc, 0, 4284);
  sdo_write_u32(slave, 0x60ff, 0, 210922);

  wkc = 0, wc = 0;
  //PDO Map set by SOD
  //JointsWrite
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

  //JointsRead
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
    error_num = 4;
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
  return 3;
}

OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
  int slave;
  (void)ptr; /* Not used */

  while (1)
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

int16 joint_is_error()
{
  uint16 SWord;
  for (int i = 0; i < joint_num; i++)
  {
    SWord = JointI[i]->status_word;
    SWord = SWord & 0x6f;
    if (SWord != 0x27)
    {
      printf("Joint %d state error, status_word:0x%x\n", i+1, JointI[i]->status_word);
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
  while (1)
  {
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    if(joint_is_error())
    {
      exit(EXIT_FAILURE);
    }

    next_time.tv_sec += (next_time.tv_nsec + cycle_time * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + cycle_time * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

int8 structural_map()
{
  int jrl = sizeof(JointsRead);
  int jwl = sizeof(JointsWrite);
  if (oloop == jwl && iloop == jrl) // 关节结构体映射
  {
    JointI[0] = (struct JointsRead *)(ec_slave[0].inputs);
    JointO[0] = (struct JointsWrite *)(ec_slave[0].outputs);
    return 0;
  }
  return 1;
}

int8 single_joint_enable(uint16 id)
{
  if (id < 1)
  {
    printf("Illegal input id %d!", id);
    return 1;
  }
  uint16 SWord;
  JointO[id - 1]->target_position = JointI[id - 1]->position_actual_value;
  JointO[id - 1]->target_velocity = 0;
  JointO[id - 1]->target_torque = 0;
  SWord = JointI[id - 1]->status_word & 0x6f;
  JointO[id - 1]->control_word = to_ctrl_word(SWord);
  ec_send_processdata();
  wkc = ec_receive_processdata(EC_TIMEOUTRET);
  if (SWord == 0x27)
  {
    printf("Joint %d enable success!\n", id);
    return 0;
  }
  else
    return 2;
}

int8 joint_enable(void)
{
  int8 result = 0;
  printf("Wait %d all joint enable...\n", joint_num);
  for (int j = 1; j <= joint_num; j++)
  {
    for (int c = 0; c < 5000; c++)
    {
      result = single_joint_enable(j);
      if (result == 0)
        break;
      osal_usleep(1000);
    }
    if (result != 0)
    {
      printf("Wait joint %d enable failed!\n", j);
      return 1;
    }
  }
  return 0;
}

int8_t ec_elmo_init(char *ifname)
{
  int16_t ret = 0;
  osal_thread_create(&thread_check, 128000, &ecatcheck, (void *)&ctime);
  ret = elmo_config(ifname);
  if (ret != 0)
  {
    return ret;
  }
  ret = structural_map();
  if (ret != 0)
  {
    printf("structural map failed, return %d\n", ret);
    return 2;
  }
  joint_enable();
  osal_thread_create_rt(&thread_sync, 204800, &sync_thread, NULL);
  if (ret != 0)
  {
    printf("joint enable failed, return %d\n", ret);
    return 3;
  }
  return 0;
}
