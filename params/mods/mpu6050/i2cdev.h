#ifndef _I2CDEV_H_
#define _I2CDEV_H_

#include <typedef.h>

#define I2C_DEV	"/dev/i2c-1"

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Status of read operation (true = success)
 */
s8 i2cdev_readBit(u8 devAddr, u8 regAddr, u8 bitNum, u8 *data);

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Status of read operation (true = success)
 */
s8 i2cdev_readBitW(u8 devAddr, u8 regAddr, u8 bitNum, u16 *data);

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Status of read operation (true = success)
 */
s8 i2cdev_readBits(u8 devAddr, u8 regAddr, u8 bitStart, u8 length, u8 *data);

/** Read multiple bits from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
s8 i2cdev_readBitsW(u8 devAddr, u8 regAddr, u8 bitStart, u8 length, u16 *data);

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Status of read operation (true = success)
 */
s8 i2cdev_readByte(u8 devAddr, u8 regAddr, u8 *data);
/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Status of read operation (true = success)
 */
s8 i2cdev_readWord(u8 devAddr, u8 regAddr, u16 *data);

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Number of bytes read (-1 indicates failure)
 */
s8 i2cdev_readBytes(u8 devAddr, u8 regAddr, u8 length, u8 *data);

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return Number of words read (0 indicates failure)
 */
s8 i2cdev_readWords(u8 devAddr, u8 regAddr, u8 length, u16 *data);

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
int i2cdev_writeBit(u8 devAddr, u8 regAddr, u8 bitNum, u8 data);

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
int i2cdev_writeBitW(u8 devAddr, u8 regAddr, u8 bitNum, u16 data);

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
int i2cdev_writeBits(u8 devAddr, u8 regAddr, u8 bitStart, u8 length, u8 data);

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
int i2cdev_writeBitsW(u8 devAddr, u8 regAddr, u8 bitStart, u8 length, u16 data);

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
int i2cdev_writeByte(u8 devAddr, u8 regAddr, u8 data);

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
int i2cdev_writeWord(u8 devAddr, u8 regAddr, u16 data);

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
int i2cdev_writeBytes(u8 devAddr, u8 regAddr, u8 length, u8* data);

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
int i2cdev_writeWords(u8 devAddr, u8 regAddr, u8 length, u16* data);

#endif
