/**
 * @file ec_elmo.c
 * @author york (york-fu@outlook.com)
 * @brief
 * @version 0.1
 * @date 2021-09-09
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <stdio.h>
#include <string.h>
#include "osal.h"
#include "ethercat.h"
#include "ec_elmo.h"

#define EC_TIMEOUTMON 500

static char IOmap[4096];
static uint8 currentgroup = 0;
static volatile int wkc;
static int expectedWKC;
static int oloop, iloop;
static boolean inOP;

static OSAL_THREAD_HANDLE thread1, thread2;
static uint8_t th_run = 0;
pthread_mutex_t mtx_pdo;

struct ELMORead *elmoI[NUM_SLAVE_MAX];
struct ELMOWrite *elmoO[NUM_SLAVE_MAX];

static MotorConfig_t *motor_cfg = NULL;

static int elmo_setup(uint16 slave)
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
    printf("Failed to PDO config, Motor %d, wkc: %d, wkc ref: %d\n", slave, wkc, wkc_ref);
    ec_close();
    exit(0);
  }

  int rdl;
  int32 i32val = 0;
  uint32 u32val = 0;
  uint16 chk;

  chk = 100;
  wkc = 0;
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
    printf("Failedto read rated current, Motor %d\n", slave);
    ec_close();
    exit(0);
  }

  chk = 100;
  wkc = 0;
  while (!wkc && (chk--))
  {
    rdl = sizeof(i32val), i32val = 0;
    wkc = ec_SDOread(slave, 0x6064, 0x00, FALSE, &rdl, &i32val, EC_TIMEOUTRXM);
    osal_usleep(1000);
  }
  if (chk != 0)
  {
    printf("Motor %d actual pos %d / %f\n", slave, i32val, (double)i32val / motor_cfg->pos_factor[slave - 1]);
  }
  else
  {
    printf("Failedto read position, Motor %d\n", slave);
  }

  motor_cfg->num++;

  return 0;
}

static int8_t elmo_map()
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
  printf("Failed to map. num: %d, wl: %d, rl %d\n", motor_cfg->num, wl * motor_cfg->num, rl * motor_cfg->num);
  return -1;
}

static int8_t check_enable()
{
  uint16_t index, sw;
  for (uint8_t i = 0; i < motor_cfg->num; i++)
  {
    if (!motor_cfg->enable[i])
    {
      continue;
    }
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

OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
  int slave;

  while (th_run)
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

OSAL_THREAD_FUNC rt_thread(void *ptr)
{
  double dt = *(double *)ptr;
  printf("Ethercat synchronization period: %f\n", dt);

  struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (th_run)
  {
    pthread_mutex_lock(&mtx_pdo);
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    pthread_mutex_unlock(&mtx_pdo);

    if (check_enable() != 0)
    {
      ec_slave[0].state = EC_STATE_INIT;
      ec_writestate(0);
      inOP = FALSE;
      ec_close();
      th_run = 0;
      exit(EXIT_FAILURE);
      return;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

int8_t elmo_init(ECMConfig_t *ec_cfg, MotorConfig_t *m_cfg)
{
  motor_cfg = m_cfg;

  if (!ec_init(ec_cfg->ifname))
  {
    printf("No socket connection on %s\nExcecute as root\n", ec_cfg->ifname);
    return -1;
  }
  printf("ec_init on %s succeeded.\n", ec_cfg->ifname);

  if (ec_config_init(FALSE) < 1)
  {
    printf("No slaves found!\n");
    ec_close();
    return -2;
  }
  printf("%d slaves found and configured.\n", ec_slavecount);

  motor_cfg->num = 0;
  if (ec_slavecount > 0)
  {
    for (uint16_t slc = 1; slc <= ec_slavecount; slc++)
    {
      if ((ec_slave[slc].eep_man == 0x9a) && (ec_slave[slc].eep_id == 0x30924))
      {
        ec_slave[slc].PO2SOconfig = &elmo_setup;
      }
    }
  }

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

  /* send one valid process data to make outputs in slaves happy*/
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);

  printf("Request operational state for all slaves\n");
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  /* request OP state for all slaves */
  ec_writestate(0);
  int chk = 100;
  /* wait for all slaves to reach OP state */
  do
  {
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
  } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  if (ec_slave[0].state != EC_STATE_OPERATIONAL)
  {
    printf("Not all slaves reached operational state.\n");
    ec_readstate();
    for (uint16_t i = 1; i <= ec_slavecount; i++)
    {
      if (ec_slave[i].state != EC_STATE_OPERATIONAL)
      {
        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
      }
    }
    ec_close();
    return -3;
  }
  printf("Operational state reached for all slaves.\n");
  inOP = TRUE;

  if (elmo_map() != 0)
  {
    return -4;
  }

  th_run = 1;
  int ret = 0;
  ret = osal_thread_create(&thread1, 128000, &ecatcheck, (void *)&ctime);
  if (ret != 1)
  {
    printf("Failed to create 'ecatcheck' thread, return %d\n", ret);
    return -5;
  }

  ret = osal_thread_create_rt(&thread2, 204800, &rt_thread, (void *)(&ec_cfg->dt));
  if (ret != 1)
  {
    printf("Failed to create 'rt_thread' thread, return %d\n", ret);
    return -6;
  }

  return 0;
}

int8_t elmo_deinit()
{
  th_run = 0;
  osal_usleep(5000);
  ec_slave[0].state = EC_STATE_INIT;
  ec_writestate(0);
  inOP = FALSE;
  ec_close();
  return 0;
}

void motor_cfg_print(MotorConfig_t *cfg, uint32_t num)
{
  printf("pos factor: [");
  for (uint16_t i = 0; i < num - 1; i++)
  {
    printf("%f, ", cfg->pos_factor[i]);
  }
  printf("%f]\n", cfg->pos_factor[num - 1]);

  printf("vel factor: [");
  for (uint16_t i = 0; i < num - 1; i++)
  {
    printf("%f, ", cfg->vel_factor[i]);
  }
  printf("%f]\n", cfg->vel_factor[num - 1]);

  printf("pos offset: [");
  for (uint16_t i = 0; i < num - 1; i++)
  {
    printf("%f, ", cfg->pos_offset[i]);
  }
  printf("%f]\n", cfg->pos_offset[num - 1]);
}