#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>

#include <i2cdev.h>

/** Default timeout value for read operations.
 * Set this to 0 to disable timeout detection.
 */
u16 timeout = 0;

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Status of read operation (1 = success)
 */
s8 i2cdev_readBit(u8 devAddr, u8 regAddr, u8 bitNum, u8 *data)
{
	u8 b;
	u8 count = i2cdev_readByte(devAddr, regAddr, &b);
	*data = b & (1 << bitNum);
	return count;
}

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Status of read operation (1 = success)
 */
s8 i2cdev_readBitW(u8 devAddr, u8 regAddr, u8 bitNum, u16 *data)
{
	u16 b;
	u8 count = i2cdev_readWord(devAddr, regAddr, &b);
	*data = b & (1 << bitNum);
	return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Status of read operation (1 = success)
 */
s8 i2cdev_readBits(u8 devAddr, u8 regAddr, u8 bitStart, u8 length, u8 *data)
{
	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//   -> 010 shifted
	u8 count, b;
	if ((count = i2cdev_readByte(devAddr, regAddr, &b)) != 0)
	{
		u8 mask = ((1 << length) - 1) << (bitStart - length + 1);
		b &= mask;
		b >>= (bitStart - length + 1);
		*data = b;
	}
	return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
s8 i2cdev_readBitsW(u8 devAddr, u8 regAddr, u8 bitStart, u8 length, u16 *data)
{
	// 1101011001101001 read byte
	// fedcba9876543210 bit numbers
	//    xxx           args: bitStart=12, length=3
	//    010           masked
	//           -> 010 shifted
	u8 count;
	u16 w;
	if ((count = i2cdev_readWord(devAddr, regAddr, &w)) != 0)
	{
		u16 mask = ((1 << length) - 1) << (bitStart - length + 1);
		w &= mask;
		w >>= (bitStart - length + 1);
		*data = w;
	}
	return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Status of read operation (1 = success)
 */
s8 i2cdev_readByte(u8 devAddr, u8 regAddr, u8 *data)
{
	return i2cdev_readBytes(devAddr, regAddr, 1, data);
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Status of read operation (1 = success)
 */
s8 i2cdev_readWord(u8 devAddr, u8 regAddr, u16 *data)
{
	return i2cdev_readWords(devAddr, regAddr, 1, data);
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Number of bytes read (-1 indicates failure)
 */
s8 i2cdev_readBytes(u8 devAddr, u8 regAddr, u8 length, u8 *data)
{
	s8 count = 0;
	int fd = open(I2C_DEV, O_RDWR);

	if (fd < 0)
	{
		fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
		return (-1);
	}
	if (ioctl(fd, I2C_SLAVE, devAddr) < 0)
	{
		fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
		close(fd);
		return (-1);
	}
	if (write(fd, &regAddr, 1) != 1)
	{
		fprintf(stderr, "Failed to write reg: %s\n", strerror(errno));
		close(fd);
		return (-1);
	}
	count = read(fd, data, length);
	if (count < 0)
	{
		//fprintf(stderr, "Failed to read device(%d): %s\n", count, ::strerror(errno));
		close(fd);
		return (-1);
	}
	else if (count != length)
	{
		fprintf(stderr, "Short read  from device, expected %d, got %d\n", length, count);
		close(fd);
		return (-1);
	}
	close(fd);

	return count;
}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Number of words read (0 indicates failure)
 */
s8 i2cdev_readWords(u8 devAddr, u8 regAddr, u8 length, u16 *data)
{
	s8 count = 0;

	printf("ReadWords() not implemented\n");
	// Use readBytes() and potential byteswap
	*data = 0; // keep the compiler quiet

	return count;
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (1 = success)
 */
int i2cdev_writeBit(u8 devAddr, u8 regAddr, u8 bitNum, u8 data)
{
	u8 b;
	i2cdev_readByte(devAddr, regAddr, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	return i2cdev_writeByte(devAddr, regAddr, b);
}

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (1 = success)
 */
int i2cdev_writeBitW(u8 devAddr, u8 regAddr, u8 bitNum, u16 data)
{
	u16 w;
	i2cdev_readWord(devAddr, regAddr, &w);
	w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
	return i2cdev_writeWord(devAddr, regAddr, w);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (1 = success)
 */
int i2cdev_writeBits(u8 devAddr, u8 regAddr, u8 bitStart, u8 length, u8 data)
{
	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	u8 b;
	if (i2cdev_readByte(devAddr, regAddr, &b) != 0)
	{
		u8 mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		b &= ~(mask); // zero all important bits in existing byte
		b |= data; // combine data with existing byte
		return i2cdev_writeByte(devAddr, regAddr, b);
	}
	else
	{
		return -1;
	}
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (1 = success)
 */
int i2cdev_writeBitsW(u8 devAddr, u8 regAddr, u8 bitStart, u8 length, u16 data)
{
	//              010 value to write
	// fedcba9876543210 bit numbers
	//    xxx           args: bitStart=12, length=3
	// 0001110000000000 mask byte
	// 1010111110010110 original value (sample)
	// 1010001110010110 original & ~mask
	// 1010101110010110 masked | value
	u16 w;
	if (i2cdev_readWord(devAddr, regAddr, &w) != 0)
	{
		u8 mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		w &= ~(mask); // zero all important bits in existing word
		w |= data; // combine data with existing word
		return i2cdev_writeWord(devAddr, regAddr, w);
	}
	else
	{
		return -1;
	}
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (1 = success)
 */
int i2cdev_writeByte(u8 devAddr, u8 regAddr, u8 data)
{
	return i2cdev_writeBytes(devAddr, regAddr, 1, &data);
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (1 = success)
 */
int i2cdev_writeWord(u8 devAddr, u8 regAddr, u16 data)
{
	return i2cdev_writeWords(devAddr, regAddr, 1, &data);
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (1 = success)
 */
int i2cdev_writeBytes(u8 devAddr, u8 regAddr, u8 length, u8* data)
{
	s8 count = 0;
	u8 buf[128];
	int fd;

	if (length > 127)
	{
		fprintf(stderr, "Byte write count (%d) > 127\n", length);
		return (0);
	}

	fd = open(I2C_DEV, O_RDWR);
	if (fd < 0)
	{
		fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
		return (0);
	}
	if (ioctl(fd, I2C_SLAVE, devAddr) < 0)
	{
		fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
		close(fd);
		return (0);
	}
	buf[0] = regAddr;
	memcpy(buf + 1, data, length);
	count = write(fd, buf, length + 1);
	if (count < 0)
	{
		//fprintf(stderr, "Failed to write device(%d): %s\n", count, ::strerror(errno));
		close(fd);
		return (0);
	}
	else if (count != length + 1)
	{
		fprintf(stderr, "Short write to device, expected %d, got %d\n", length + 1, count);
		close(fd);
		return (0);
	}
	close(fd);

	return 1;
}

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (1 = success)
 */
int i2cdev_writeWords(u8 devAddr, u8 regAddr, u8 length, u16* data)
{
	s8 count = 0;
	u8 buf[128];
	int i, fd;

	// Should do potential byteswap and call writeBytes() really, but that
	// messes with the callers buffer

	if (length > 63)
	{
		fprintf(stderr, "Word write count (%d) > 63\n", length);
		return (0);
	}

	fd = open(I2C_DEV, O_RDWR);
	if (fd < 0)
	{
		fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
		return (0);
	}
	if (ioctl(fd, I2C_SLAVE, devAddr) < 0)
	{
		fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
		close(fd);
		return (0);
	}
	buf[0] = regAddr;
	for (i = 0; i < length; i++)
	{
		buf[i * 2 + 1] = data[i] >> 8;
		buf[i * 2 + 2] = data[i];
	}
	count = write(fd, buf, length * 2 + 1);
	if (count < 0)
	{
		//fprintf(stderr, "Failed to write device(%d): %s\n", count, ::strerror(errno));
		close(fd);
		return (0);
	}
	else if (count != length * 2 + 1)
	{
		fprintf(stderr, "Short write to device, expected %d, got %d\n", length + 1, count);
		close(fd);
		return (0);
	}
	close(fd);
	return 1;
}

