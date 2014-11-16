/* ST L3GD20 Gyroscope Driver */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <assert.h>
#include <sys/types.h>
#include <string.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sensors/l3gd20.h>

#ifdef CONFIG_L3GD20

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Addreses ********************************************************/

#define ST_L3GD20_WHO_AM_I      0x0f
#define ST_L3GD20_CTRL_REG1     0x20
#define ST_L3GD20_CTRL_REG2     0x21
#define ST_L3GD20_CTRL_REG3     0x22
#define ST_L3GD20_CTRL_REG4     0x23
#define ST_L3GD20_CTRL_REG5     0x24
#define ST_L3GD20_REFERENCE     0x25
#define ST_L3GD20_OUT_TEMP      0x26
#define ST_L3GD20_STATUS_REG    0x27
#define ST_L3GD20_OUT_X_L       0x28
#define ST_L3GD20_OUT_X_H       0x29
#define ST_L3GD20_OUT_Y_L       0x2A
#define ST_L3GD20_OUT_Y_H       0x2B
#define ST_L3GD20_OUT_Z_L       0x2C
#define ST_L3GD20_OUT_Z_H       0x2D
#define ST_L3GD20_FIFO_CTRL_REG 0x2E
#define ST_L3GD20_FIFO_SRC_REG  0x2F
#define ST_L3GD20_INT1_CFG      0x30
#define ST_L3GD20_INT1_SRC      0x31
#define ST_L3GD20_INT1_THS_XH   0x32
#define ST_L3GD20_INT1_THS_XL   0x33
#define ST_L3GD20_INT1_THS_YH   0x34
#define ST_L3GD20_INT1_THS_YL   0x35
#define ST_L3GD20_INT1_THS_ZH   0x36
#define ST_L3GD20_INT1_THS_ZL   0x37
#define ST_L3GD20_INT1_DURATION 0x38

/* Register Bits and Fields *************************************************/

/* CTRL_REG1 ---------------------------------------------------------------*/

/* Output Data Rate selection */

#define ST_L3GD20_CTRL_REG1_DR_MASK  3
#define ST_L3GD20_CTRL_REG1_DR_SHIFT 6

/* Bandwidth selection */

#define ST_L3GD20_CTRL_REG1_BW_MASK  3
#define ST_L3GD20_CTRL_REG1_BW_SHIFT 4

#define ST_L3GD20_CTRL_REG1_PD  (1 << 3)
#define ST_L3GD20_CTRL_REG1_Zen (1 << 2)
#define ST_L3GD20_CTRL_REG1_Xen (1 << 1)
#define ST_L3GD20_CTRL_REG1_Yen (1 << 0)

/* CTRL_REG2 ---------------------------------------------------------------*/

/* High-pass filter mode selection */

#define ST_L3GD20_CTRL_REG2_HPM_MASK 0x30
#define ST_L3GD20_CTRL_REG2_HPM_SHIFT 4

/* High-pass filter cut off frequency configuration */

#define ST_L3GD20_CTRL_REG2_HPCF_MASK 0x0F
#define ST_L3GD20_CTRL_REG2_HPCF_SHIFT 0

/* CTRL_REG3 ---------------------------------------------------------------*/

#define ST_L3GD20_CTRL_REG3_I1_Int1    (1 << 7) /* Interrupt enable on INT1 pin */
#define ST_L3GD20_CTRL_REG3_I1_Boot    (1 << 6) /* Boot status available on INT1 */
#define ST_L3GD20_CTRL_REG3_H_Lactive  (1 << 5) /* Interrupt active configuration on INT1. */
#define ST_L3GD20_CTRL_REG3_PP_OD      (1 << 4) /* Push-pull / Open drain */
#define ST_L3GD20_CTRL_REG3_I2_DRDY    (1 << 3) /* Date-ready on DRDY/INT2 */
#define ST_L3GD20_CTRL_REG3_I2_WTM     (1 << 2) /* FIFO watermark interrupt on DRDY/INT2 */
#define ST_L3GD20_CTRL_REG3_I2_ORun    (1 << 1) /* FIFO overrun interrupt on DRDY/INT2 */
#define ST_L3GD20_CTRL_REG3_I2_Empty   (1 << 0) /* FIFO empty interrupt on DRDY/INT2 */

/* CTRL_REG4 ---------------------------------------------------------------*/

#define ST_L3GD20_CTRL_REG4_BDU (1 << 7) /* Block data update */
#define ST_L3GD20_CTRL_REG4_BLE (1 << 6) /* Big/little endian data selection */

/* Full scale selection */
#define ST_L3GD20_CTRL_REG4_FS_MASK 0x30
#define ST_L3GD20_CTRL_REG4_FS_SHIFT 4

#define ST_L3GD20_CTRL_REG4_SIM (1 << 0) /* SPI serial interface mode selection */

/* CTRL_REG5 ---------------------------------------------------------------*/

#define ST_L3GD20_CTRL_REG5_BOOT    (7 << 1) /* Reboot memory content */
#define ST_L3GD20_CTRL_REG5_FIFO_EN (1 << 6) /* FIFO enable */
#define ST_L3GD20_CTRL_REG5_HPen    (1 << 4) /* High-pass filter enable */

/* INT1 selection configuration */
#define ST_L3GD20_CTRL_REG5_INT1_Sel_MASK  0x0B
#define ST_L3GD20_CTRL_REG5_INT1_Sel_SHIFT 2

/* Out selection configuration */
#define ST_L3GD20_CTRL_REG5_Out_Sel_MASK   0x03
#define ST_L3GD20_CTRL_REG5_Out_Sel_SHIFT  0

/* STATUS_REG --------------------------------------------------------------*/

#define ST_L3GD20_STATUS_REG_ZYXOR (1 << 7) /* X, Y, Z-axis overrun */
#define ST_L3GD20_STATUS_REG_ZOR   (1 << 6) /* Z axis data overrun */
#define ST_L3GD20_STATUS_REG_YOR   (1 << 5) /* Y axis data overrun */
#define ST_L3GD20_STATUS_REG_XOR   (1 << 4) /* X axis data overrun */
#define ST_L3GD20_STATUS_REG_ZYXDA (1 << 3) /* X, Y, Z-axis data available */
#define ST_L3GD20_STATUS_REG_ZDA   (1 << 2) /* Z axis new data available */
#define ST_L3GD20_STATUS_REG_YDA   (1 << 1) /* Y axis new data available */
#define ST_L3GD20_STATUS_REG_XDA   (1 << 0) /* X axis new data available */

/* FIFO_CTRL_REG -----------------------------------------------------------*/

/* FIFO mode selection */
#define ST_L3GD20_FIFO_CTRL_REG_FM_MASK         0xE0
#define ST_L3GD20_FIFO_CTRL_REG_FM_SHIFT        5

/* FIFO threshold */
#define ST_L3GD20_FIFO_CTRL_REG_WTM_MASK        0x1F
#define ST_L3GD20_FIFO_CTRL_REG_WTM_SHIFT       0

/* FIFO_SRC_REG ------------------------------------------------------------*/

#define ST_L3GD20_FIFO_SRC_REG_WTM   (1 << 7) /* Watermark status */
#define ST_L3GD20_FIFO_SRC_REG_OVRN  (1 << 6) /* Overrun bit status */
#define ST_L3GD20_FIFO_SRC_REG_EMPTY (1 << 5) /* FIFO empty bit */

/* FIFO stored data level */
#define ST_L3GD20_FIFO_SRC_REG_FSS_MASK  0x1F
#define ST_L3GD20_FIFO_SRC_REG_FSS_SHIFT 0

/* INT1_CFG ----------------------------------------------------------------*/

#define ST_L3GD20_INT1_CFG_AND_OR (1 << 7) /* AND/OR combination of interrupt events */
#define ST_L3GD20_INT1_CFG_LIR    (1 << 6) /* Latch interrupt request */
#define ST_L3GD20_INT1_CFG_ZHIE   (1 << 5) /* Enable interrupt generation on Z high event */
#define ST_L3GD20_INT1_CFG_ZLIE   (1 << 4) /* Enable interrupt generation on Z low evet */
#define ST_L3GD20_INT1_CFG_YHIE   (1 << 3) /* Enable interrupt generation on Y high event */
#define ST_L3GD20_INT1_CFG_YLIE   (1 << 2) /* Enable interrupt generation on Y low event */
#define ST_L3GD20_INT1_CFG_XHIE   (1 << 1) /* Enable interrupt generation on X high event */
#define ST_L3GD20_INT1_CFG_XLIE   (1 << 0) /* Enable interrupt generation on X low event */

/* INT1_SRC ----------------------------------------------------------------*/

#define ST_L3GD20_INT1_SRC_IA (1 << 6) /* Interrupt active */
#define ST_L3GD20_INT1_SRC_ZH (1 << 5) /* Z high */
#define ST_L3GD20_INT1_SRC_ZL (1 << 4) /* Z low */
#define ST_L3GD20_INT1_SRC_YH (1 << 3) /* Y high */
#define ST_L3GD20_INT1_SRC_YL (1 << 2) /* Y low */
#define ST_L3GD20_INT1_SRC_XH (1 << 1) /* X high */
#define ST_L3GD20_INT1_SRC_XL (1 << 0) /* X low */

/* INT1_DURATION -----------------------------------------------------------*/

#define ST_L3GD20_INT1_DURATION_WAIT (1 << 7) /* WAIT enable */

/* Duration value */
#define ST_L3GD20_INT1_DURATION_D_MASK  0x7F
#define ST_L3GD20_INT1_DURATION_D_SHIFT 0

/* Device identification register value */

#define ST_L3GD20_WHO_AM_I_VALUE 0x4D

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct l3gd20_dev_s
{
  struct i2c_dev_s       *i2c;      /* I2C bus driver instance */
  uint16_t                address;  /* I2C slave adress */
  uint8_t                 cr1;
  uint8_t                 cr2;
  uint8_t                 cr3;
  uint8_t                 cr4;
  uint8_t                 cr5;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int l3gd20_access(FAR struct l3gd20_dev_s *dev, uint8_t subaddr,
                         FAR uint8_t *buf, int length)
{
  uint16_t flags = 0;
  int retval;

  if (length > 0)
    {
      flags = I2C_M_READ;
    }
  else
    {
      flags = I2C_M_NORESTART;
      length = -length;
    }

  /* TODO Validate subaddress */

  if (subaddr < ST_L3GD20_WHO_AM_I || subaddr > ST_L3GD20_INT1_DURATION)
    {
      errno = EFAULT;
      return ERROR;
    }

  /* Create message and send */

  struct i2c_msg_s msgv[2] =
  {
    {
      .addr   = dev->address,
      .flags  = 0,
      .buffer = &subaddr,
      .length = 1,
    },
    {
      .addr   = dev->address,
      .flags  = flags,
      .buffer = buf,
      .length = length
    }
  };

  if ((retval = I2C_TRANSFER(dev->i2c, msgv, 2)) == OK)
    {
      return length;
    }

  return retval;
}

static int l3gd20_readregs(FAR struct l3gd20_dev_s *dev)
{
  if (l3gd20_access(dev, ST_L3GD20_CTRL_REG1, &dev->cr1, 5) != 5)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct l3gd20_dev_s * l3gd20_init(FAR struct i2c_dev_s * i2c,
                                      uint16_t address)
{
  FAR struct l3gd20_dev_s *dev;
  uint8_t retval;

  ASSERT(i2c);
  ASSERT(address);

  dev = kmm_malloc(sizeof(struct l3gd20_dev_s));
  if (dev == NULL)
    {
      errno = ENOMEM;
      return NULL;
    }

  memset(dev, 0, sizeof(struct l3gd20_dev_s));
  dev->i2c     = i2c;
  dev->address = address;

  /* Probe device */

  if (l3gd20_access(dev, ST_L3GD20_WHO_AM_I, &retval, 1) > 0)
    {
      if (retval == ST_L3GD20_WHO_AM_I_VALUE)
        {
          if (l3gd20_readregs(dev) == OK && l3gd20_powerup(dev) == OK)
            {
              errno = 0;
              return dev;
            }
        }
      retval = ENODEV;
    }
  else
    {
      retval = EFAULT;
    }

  kmm_free(dev);
  errno = retval;
  return NULL;
}

int l3gd20_deinit(struct l3gd20_dev_s *dev)
{
  ASSERT(dev);

  kmm_free(dev);

  return OK;
}

int l3gd20_powerup(struct l3gd20_dev_s *dev)
{
  return ERROR;
}

int l3gd20_powerdown(struct l3gd20_dev_s *dev)
{
  return ERROR;
}

int l3gd20_setfifomode(struct l3gd20_dev_s *dev, enum l3gd20_fifomode_e mode)
{
  ASSERT(dev);

  /* TODO Set mode in FIFO_CTRL_REG */

  return ERROR;
}

int l3gd20_getfifomode(struct l3gd20_dev_s *dev, enum l3gd20_fifomode_e *mode)
{
  const uint8_t regaddr = ST_L3GD20_FIFO_CTRL_REG;
  const uint8_t mask    = ST_L3GD20_FIFO_CTRL_REG_FM_MASK;
  const uint8_t shift   = ST_L3GD20_FIFO_CTRL_REG_FM_SHIFT;
  uint8_t regval;

  ASSERT(dev);

  if (l3gd20_access(dev, regaddr, &regval, 1) != 1)
    return ERROR;

  /* Extract the FIFO mode from the register value */
  *mode = (regval & mask) >> shift;

  return OK;
}

int l3gd20_setfifoths(struct l3gd20_dev_s *dev, uint8_t ths)
{
  return OK;
}

int l3gd20_getfifoths(struct l3gd20_dev_s *dev, uint8_t *ths)
{
  const uint8_t regaddr = ST_L3GD20_FIFO_CTRL_REG;
  const uint8_t mask    = ST_L3GD20_FIFO_CTRL_REG_WTM_MASK;
  const uint8_t shift   = ST_L3GD20_FIFO_CTRL_REG_WTM_SHIFT;
  uint8_t regval;

  ASSERT(dev);

  if (l3gd20_access(dev, regaddr, &regval, 1) != 1)
    return ERROR;

  *ths = (regval & mask) >> shift;

  return OK;
}

int l3gd20_setthresholds(FAR struct l3gd20_vector_s *vect)
{
  return ERROR;
}

int l3gd20_getthresholds(FAR struct l3gd20_vector_s *vect)
{
  return ERROR;
}

FAR const struct l3gd20_vector_s *
l3gd20_getreadings(FAR struct l3gd20_dev_s *dev)
{
  struct l3gd20_vector_s *vect;

  ASSERT(dev);

  /* TODO Implement read address to samples */

  return NULL;
}

#endif /* CONFIG_L3GD20 */
