#include <MLX90640_I2C_Driver.h>

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#ifndef MLX90640_I2C_DEVICE
#define MLX90640_I2C_DEVICE "/dev/i2c-1"
#endif

#define MLX90640_I2C_MAX_READ_WORDS 128
#define MLX90640_I2C_RETRIES 3

static int i2c_fd = -1;
static int configured_freq = 100000;

static int ensure_i2c_open(void)
{
    const char *device;

    if (i2c_fd >= 0)
    {
        return MLX90640_NO_ERROR;
    }

    device = getenv("MLX90640_I2C_DEVICE");
    if ((device == NULL) || (device[0] == '\0'))
    {
        device = MLX90640_I2C_DEVICE;
    }

    i2c_fd = open(device, O_RDWR);
    if (i2c_fd < 0)
    {
        return -MLX90640_I2C_NACK_ERROR;
    }

    return MLX90640_NO_ERROR;
}

static int i2c_transfer_with_retries(struct i2c_rdwr_ioctl_data *transfer)
{
    int error;

    for (int attempt = 0; attempt < MLX90640_I2C_RETRIES; attempt++)
    {
        error = ioctl(i2c_fd, I2C_RDWR, transfer);
        if (error == (int)transfer->nmsgs)
        {
            return MLX90640_NO_ERROR;
        }

        if ((errno != EAGAIN) && (errno != EINTR) && (errno != ETIMEDOUT))
        {
            break;
        }
    }

    return -MLX90640_I2C_NACK_ERROR;
}

void MLX90640_I2CInit(void)
{
    (void)ensure_i2c_open();
}

int MLX90640_I2CGeneralReset(void)
{
    uint8_t command = 0x06;
    struct i2c_msg messages[1];
    struct i2c_rdwr_ioctl_data transfer;
    int error;

    error = ensure_i2c_open();
    if (error != MLX90640_NO_ERROR)
    {
        return error;
    }

    messages[0].addr = 0x00;
    messages[0].flags = 0;
    messages[0].len = 1;
    messages[0].buf = &command;

    transfer.msgs = messages;
    transfer.nmsgs = 1;

    return i2c_transfer_with_retries(&transfer);
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
    uint16_t wordsRead = 0;
    int error;

    if ((nMemAddressRead > 0) && (data == NULL))
    {
        return -MLX90640_I2C_NACK_ERROR;
    }

    error = ensure_i2c_open();
    if (error != MLX90640_NO_ERROR)
    {
        return error;
    }

    while (wordsRead < nMemAddressRead)
    {
        uint16_t wordsThisRead = nMemAddressRead - wordsRead;
        uint16_t registerAddress = startAddress + wordsRead;
        uint8_t addressBytes[2];
        uint8_t readBuffer[MLX90640_I2C_MAX_READ_WORDS * 2];
        struct i2c_msg messages[2];
        struct i2c_rdwr_ioctl_data transfer;

        if (wordsThisRead > MLX90640_I2C_MAX_READ_WORDS)
        {
            wordsThisRead = MLX90640_I2C_MAX_READ_WORDS;
        }

        addressBytes[0] = (uint8_t)(registerAddress >> 8);
        addressBytes[1] = (uint8_t)(registerAddress & 0x00FF);

        messages[0].addr = slaveAddr;
        messages[0].flags = 0;
        messages[0].len = 2;
        messages[0].buf = addressBytes;

        messages[1].addr = slaveAddr;
        messages[1].flags = I2C_M_RD;
        messages[1].len = wordsThisRead * 2;
        messages[1].buf = readBuffer;

        transfer.msgs = messages;
        transfer.nmsgs = 2;

        error = i2c_transfer_with_retries(&transfer);
        if (error != MLX90640_NO_ERROR)
        {
            return error;
        }

        for (uint16_t i = 0; i < wordsThisRead; i++)
        {
            data[wordsRead + i] = ((uint16_t)readBuffer[2 * i] << 8) | readBuffer[(2 * i) + 1];
        }

        wordsRead += wordsThisRead;
    }

    return MLX90640_NO_ERROR;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    uint8_t writeBuffer[4];
    struct i2c_msg messages[1];
    struct i2c_rdwr_ioctl_data transfer;
    int error;

    error = ensure_i2c_open();
    if (error != MLX90640_NO_ERROR)
    {
        return error;
    }

    writeBuffer[0] = (uint8_t)(writeAddress >> 8);
    writeBuffer[1] = (uint8_t)(writeAddress & 0x00FF);
    writeBuffer[2] = (uint8_t)(data >> 8);
    writeBuffer[3] = (uint8_t)(data & 0x00FF);

    messages[0].addr = slaveAddr;
    messages[0].flags = 0;
    messages[0].len = sizeof(writeBuffer);
    messages[0].buf = writeBuffer;

    transfer.msgs = messages;
    transfer.nmsgs = 1;

    error = i2c_transfer_with_retries(&transfer);
    if (error != MLX90640_NO_ERROR)
    {
        return -MLX90640_I2C_WRITE_ERROR;
    }

    return MLX90640_NO_ERROR;
}

void MLX90640_I2CFreqSet(int freq)
{
    configured_freq = freq;
    (void)configured_freq;
}
