package com.lobstahbots.nfclib;

import java.nio.ByteBuffer;
import java.util.Optional;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

/**
 * A class for controlling an RC522 RFID reader. See the datasheet at
 * https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf. Based off of
 * https://github.com/miguelbalboa/rfid/.
 */
public class RC522 {
    /**
     * MFRC522 accepts up to 10MHz, set to 4MHz
     */
    public static final int SPI_CLOCK = 4000000;
    /**
     * FIFO size is 64 bytes
     */
    public static final int FIFO_SIZE = 64;
    // Firmware data for self-test
    // Reference values based on firmware version
    /**
     * Version 0.0 (0x90)
     * <p>
     * Philips Semiconductors; Preliminary Specification Revision 2.0 - 01 August
     * 2005; 16.1 self-test
     */
    public static final ByteBuffer RC522_FIRMWARE_REFERENCE_V0_0;
    /**
     * Version 1.0 (0x91)
     * <p>
     * NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
     */
    public static final ByteBuffer RC522_FIRMWARE_REFERENCE_V1_0;
    /**
     * Version 2.0 (0x92)
     * <p>
     * NXP Semiconductors; Rev 3.8 - 17 September 2014; 16.1.1. self-test
     */
    public static final ByteBuffer RC522_FIRMWARE_REFERENCE_V2_0;
    /**
     * Clone
     * <p>
     * Fudan Semiconductor FM17522 (0x88)
     */
    public static final ByteBuffer FM17522_FIRMWARE_REFERENCE;
    static {
        int[] RC522_FIRMWARE_REFERENCE_V0_0_ARRAY = { 0x00, 0x87, 0x98, 0x0f, 0x49, 0xFF, 0x07, 0x19, 0xBF, 0x22, 0x30,
                0x49, 0x59, 0x63, 0xAD, 0xCA, 0x7F, 0xE3, 0x4E, 0x03, 0x5C, 0x4E, 0x49, 0x50, 0x47, 0x9A, 0x37, 0x61,
                0xE7, 0xE2, 0xC6, 0x2E, 0x75, 0x5A, 0xED, 0x04, 0x3D, 0x02, 0x4B, 0x78, 0x32, 0xFF, 0x58, 0x3B, 0x7C,
                0xE9, 0x00, 0x94, 0xB4, 0x4A, 0x59, 0x5B, 0xFD, 0xC9, 0x29, 0xDF, 0x35, 0x96, 0x98, 0x9E, 0x4F, 0x30,
                0x32, 0x8D };
        RC522_FIRMWARE_REFERENCE_V0_0 = ByteBuffer.allocate(RC522_FIRMWARE_REFERENCE_V0_0_ARRAY.length);
        for (int val : RC522_FIRMWARE_REFERENCE_V0_0_ARRAY) {
            RC522_FIRMWARE_REFERENCE_V0_0.put((byte) val);
        }
        RC522_FIRMWARE_REFERENCE_V0_0.flip();
        int[] RC522_FIRMWARE_REFERENCE_V1_0_ARRAY = { 0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C, 0xC2, 0xD8, 0x7C,
                0x4D, 0xD9, 0x70, 0xC7, 0x73, 0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A, 0x14, 0xAF, 0x30, 0x61,
                0xC9, 0x70, 0xDB, 0x2E, 0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC, 0x22, 0xBC, 0xD3, 0x72, 0x35,
                0xCD, 0xAA, 0x41, 0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02, 0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D,
                0x29, 0x79 };
        RC522_FIRMWARE_REFERENCE_V1_0 = ByteBuffer.allocate(RC522_FIRMWARE_REFERENCE_V1_0_ARRAY.length);
        for (int val : RC522_FIRMWARE_REFERENCE_V1_0_ARRAY) {
            RC522_FIRMWARE_REFERENCE_V1_0.put((byte) val);
        }
        RC522_FIRMWARE_REFERENCE_V1_0.flip();
        int[] RC522_FIRMWARE_REFERENCE_V2_0_ARRAY = { 0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95, 0xD0, 0xE3, 0x0D,
                0x3D, 0x27, 0x89, 0x5C, 0xDE, 0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82, 0x51, 0x3A, 0xEB, 0x02,
                0x0C, 0xA5, 0x00, 0x49, 0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81, 0x5D, 0x48, 0x76, 0xD5, 0x71,
                0x61, 0x21, 0xA9, 0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D, 0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95,
                0x3B, 0x2F };
        RC522_FIRMWARE_REFERENCE_V2_0 = ByteBuffer.allocate(RC522_FIRMWARE_REFERENCE_V2_0_ARRAY.length);
        for (int val : RC522_FIRMWARE_REFERENCE_V2_0_ARRAY) {
            RC522_FIRMWARE_REFERENCE_V2_0.put((byte) val);
        }
        RC522_FIRMWARE_REFERENCE_V2_0.flip();
        int[] FM17522_FIRMWARE_REFERENCE_ARRAY = { 0x00, 0xD6, 0x78, 0x8C, 0xE2, 0xAA, 0x0C, 0x18, 0x2A, 0xB8, 0x7A,
                0x7F, 0xD3, 0x6A, 0xCF, 0x0B, 0xB1, 0x37, 0x63, 0x4B, 0x69, 0xAE, 0x91, 0xC7, 0xC3, 0x97, 0xAE, 0x77,
                0xF4, 0x37, 0xD7, 0x9B, 0x7C, 0xF5, 0x3C, 0x11, 0x8F, 0x15, 0xC3, 0xD7, 0xC1, 0x5B, 0x00, 0x2A, 0xD0,
                0x75, 0xDE, 0x9E, 0x51, 0x64, 0xAB, 0x3E, 0xE9, 0x15, 0xB5, 0xAB, 0x56, 0x9A, 0x98, 0x82, 0x26, 0xEA,
                0x2A, 0x62 };
        FM17522_FIRMWARE_REFERENCE = ByteBuffer.allocate(FM17522_FIRMWARE_REFERENCE_ARRAY.length);
        for (int val : FM17522_FIRMWARE_REFERENCE_ARRAY) {
            FM17522_FIRMWARE_REFERENCE.put((byte) val);
        }
        FM17522_FIRMWARE_REFERENCE.flip();
    }

    /**
     * MFRC522 registers. Described in chapter 9 of the datasheet.
     * <p>
     * When using SPI all addresses are shifted one bit left in the "SPI address
     * byte" (section 8.1.2.3)
     */
    public static enum PCDRegister {
        // Page 0: Command and status
        //						  0x00			// reserved for future use
        /**
         * starts and stops command execution
         */
        CommandReg(0x01 << 1),
        /**
         * enable and disable interrupt request control bits
         */
        ComIEnReg(0x02 << 1),
        /**
         * enable and disable interrupt request control bits
         */
        DivIEnReg(0x03 << 1),
        /**
         * interrupt request bits
         */
        ComIrqReg(0x04 << 1),
        /**
         * interrupt request bits
         */
        DivIrqReg(0x05 << 1),
        /**
         * error bits showing the error status of the last command executed
         */
        ErrorReg(0x06 << 1),
        /**
         * communication status bits
         */
        Status1Reg(0x07 << 1),
        /**
         * receiver and transmitter status bits
         */
        Status2Reg(0x08 << 1),
        /**
         * input and output of 64 byte FIFO buffer
         */
        FIFODataReg(0x09 << 1),
        /**
         * number of bytes stored in the FIFO buffer
         */
        FIFOLevelReg(0x0A << 1),
        /**
         * level for FIFO underflow and overflow warning
         */
        WaterLevelReg(0x0B << 1),
        /**
         * miscellaneous control registers
         */
        ControlReg(0x0C << 1),
        /**
         * adjustments for bit-oriented frames
         */
        BitFramingReg(0x0D << 1),
        /**
         * bit position of the first bit-collision detected on the RF interface
         */
        CollReg(0x0E << 1),
        //						  0x0F			// reserved for future use

        // Page 1: Command
        // 						  0x10			// reserved for future use
        /**
         * defines general modes for transmitting and receiving
         */
        ModeReg(0x11 << 1),
        /**
         * defines transmission data rate and framing
         */
        TxModeReg(0x12 << 1),
        /**
         * defines reception data rate and framing
         */
        RxModeReg(0x13 << 1),
        /**
         * controls the logical behavior of the antenna driver pins TX1 and TX2
         */
        TxControlReg(0x14 << 1),
        /**
         * controls the setting of the transmission modulation
         */
        TxASKReg(0x15 << 1),
        /**
         * selects the internal sources for the antenna driver
         */
        TxSelReg(0x16 << 1),
        /**
         * selects internal receiver settings
         */
        RxSelReg(0x17 << 1),
        /**
         * selects thresholds for the bit decoder
         */
        RxThresholdReg(0x18 << 1),
        /**
         * defines demodulator settings
         */
        DemodReg(0x19 << 1),
        // 						  0x1A			// reserved for future use
        // 						  0x1B			// reserved for future use
        /**
         * controls some MIFARE communication transmit parameters
         */
        MfTxReg(0x1C << 1),
        /**
         * controls some MIFARE communication receive parameters
         */
        MfRxReg(0x1D << 1),
        // 						  0x1E			// reserved for future use
        /**
         * selects the speed of the serial UART interface
         */
        SerialSpeedReg(0x1F << 1),

        // Page 2: Configuration
        // 						  0x20			// reserved for future use
        /**
         * shows the MSB and LSB values of the CRC calculation
         */
        CRCResultRegH(0x21 << 1), CRCResultRegL(0x22 << 1),
        // 						  0x23			// reserved for future use
        /**
         * controls the ModWidth setting?
         */
        ModWidthReg(0x24 << 1),
        // 						  0x25			// reserved for future use
        /**
         * configures the receiver gain
         */
        RFCfgReg(0x26 << 1),
        /**
         * selects the conductance of the antenna driver pins TX1 and TX2 for modulation
         */
        GsNReg(0x27 << 1),
        /**
         * defines the conductance of the p-driver output during periods of no
         * modulation
         */
        CWGsPReg(0x28 << 1),
        /**
         * defines the conductance of the p-driver output during periods of modulation
         */
        ModGsPReg(0x29 << 1),
        /**
         * defines settings for the internal timer
         */
        TModeReg(0x2A << 1),
        /**
         * the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
         */
        TPrescalerReg(0x2B << 1),
        /**
         * defines the 16-bit timer reload value
         */
        TReloadRegH(0x2C << 1), TReloadRegL(0x2D << 1),
        /**
         * shows the 16-bit timer value
         */
        TCounterValueRegH(0x2E << 1), TCounterValueRegL(0x2F << 1),

        // Page 3: Test Registers
        // 						  0x30			// reserved for future use
        /**
         * general test signal configuration
         */
        TestSel1Reg(0x31 << 1),
        /**
         * general test signal configuration
         */
        TestSel2Reg(0x32 << 1),
        /**
         * enables pin output driver on pins D1 to D7
         */
        TestPinEnReg(0x33 << 1),
        /**
         * defines the values for D1 to D7 when it is used as an I/O bus
         */
        TestPinValueReg(0x34 << 1),
        /**
         * shows the status of the internal test bus
         */
        TestBusReg(0x35 << 1),
        /**
         * controls the digital self-test
         */
        AutoTestReg(0x36 << 1),
        /**
         * shows the software version
         */
        VersionReg(0x37 << 1),
        /**
         * controls the pins AUX1 and AUX2
         */
        AnalogTestReg(0x38 << 1),
        /**
         * defines the test value for TestDAC1
         */
        TestDAC1Reg(0x39 << 1),
        /**
         * defines the test value for TestDAC2
         */
        TestDAC2Reg(0x3A << 1),
        /**
         * shows the value of ADC I and Q channels
         */
        TestADCReg(0x3B << 1);
        // 						  0x3C			// reserved for production tests
        // 						  0x3D			// reserved for production tests
        // 						  0x3E			// reserved for production tests
        // 						  0x3F			// reserved for production tests

        public final byte value;

        private PCDRegister(int val) {
            value = (byte) val;
        }
    };

    /**
     * MFRC522 commands. Described in chapter 10 of the datasheet.
     */
    public static enum PCDCommand {
        /**
         * no action, cancels current command execution
         */
        PCD_Idle(0x00),
        /**
         * stores 25 bytes into the internal buffer
         */
        PCD_Mem(0x01),
        /**
         * generates a 10-byte random ID number
         */
        PCD_GenerateRandomID(0x02),
        /**
         * activates the CRC coprocessor or performs a self-test
         */
        PCD_CalcCRC(0x03),
        /**
         * transmits data from the FIFO buffer
         */
        PCD_Transmit(0x04),
        /**
         * no command change, can be used to modify the CommandReg register bits without
         * affecting the command, for example, the PowerDown bit
         */
        PCD_NoCmdChange(0x07),
        /**
         * activates the receiver circuits
         */
        PCD_Receive(0x08),
        /**
         * transmits data from FIFO buffer to antenna and automatically activates the
         * receiver after transmission
         */
        PCD_Transceive(0x0C),
        /**
         * performs the MIFARE standard authentication as a reader
         */
        PCD_MFAuthent(0x0E),
        /**
         * resets the MFRC522
         */
        PCD_SoftReset(0x0F);

        public final byte value;

        private PCDCommand(int val) {
            value = (byte) val;
        }
    };

    /**
     * MFRC522 RxGain[2:0] masks, defines the receiver's signal voltage gain factor
     * (on the PCD).
     * <p>
     * Described in 9.3.3.6 / table 98 of the datasheet at
     * http://www.nxp.com/documents/data_sheet/MFRC522.pdf
     */
    public static enum PCDRxGain {
        /**
         * 000b - 18 dB, minimum
         */
        RxGain_18dB(0x00 << 4),
        /**
         * 001b - 23 dB
         */
        RxGain_23dB(0x01 << 4),
        /**
         * 010b - 18 dB, it seems 010b is a duplicate for 000b
         */
        RxGain_18dB_2(0x02 << 4),
        /**
         * 011b - 23 dB, it seems 011b is a duplicate for 001b
         */
        RxGain_23dB_2(0x03 << 4),
        /**
         * 100b - 33 dB, average, and typical default
         */
        RxGain_33dB(0x04 << 4),
        /**
         * 101b - 38 dB
         */
        RxGain_38dB(0x05 << 4),
        /**
         * 110b - 43 dB
         */
        RxGain_43dB(0x06 << 4),
        /**
         * 111b - 48 dB, maximum
         */
        RxGain_48dB(0x07 << 4),
        /**
         * 000b - 18 dB, minimum, convenience for RxGain_18dB
         */
        RxGain_min(0x00 << 4),
        /**
         * 100b - 33 dB, average, convenience for RxGain_33dB
         */
        RxGain_avg(0x04 << 4),
        /**
         * 111b - 48 dB, maximum, convenience for RxGain_48dB
         */
        RxGain_max(0x07 << 4);

        public final byte value;

        private PCDRxGain(int val) {
            value = (byte) val;
        }
    };

    /**
     * Commands sent to the PICC.
     */
    public static enum PICCCommand {
        // The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
        /**
         * REQuest command, Type A. Invites PICCs in state IDLE to go to READY and
         * prepare for anticollision or selection. 7 bit frame.
         */
        PICC_CMD_REQA(0x26),
        /**
         * Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to
         * READY(*) and prepare for anticollision or selection. 7 bit frame.
         */
        PICC_CMD_WUPA(0x52),
        /**
         * Cascade Tag. Not really a command, but used during anti collision.
         */
        PICC_CMD_CT(0x88),
        /**
         * Anti collision/Select, Cascade Level 1
         */
        PICC_CMD_SEL_CL1(0x93),
        /**
         * Anti collision/Select, Cascade Level 2
         */
        PICC_CMD_SEL_CL2(0x95),
        /**
         * Anti collision/Select, Cascade Level 3
         */
        PICC_CMD_SEL_CL3(0x97),
        /**
         * HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
         */
        PICC_CMD_HLTA(0x50),
        /**
         * Request command for Answer To Reset.
         */
        PICC_CMD_RATS(0xE0),
        // The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
        // Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
        // The read/write commands can also be used for MIFARE Ultralight.
        /**
         * Perform authentication with Key A
         */
        PICC_CMD_MF_AUTH_KEY_A(0x60),
        /**
         * Perform authentication with Key B
         */
        PICC_CMD_MF_AUTH_KEY_B(0x61),
        /**
         * Reads one 16 byte block from the authenticated sector of the PICC. Also used
         * for MIFARE Ultralight.
         */
        PICC_CMD_MF_READ(0x30),
        /**
         * Writes one 16 byte block to the authenticated sector of the PICC. Called
         * "COMPATIBILITY WRITE" for MIFARE Ultralight.
         */
        PICC_CMD_MF_WRITE(0xA0),
        /**
         * Decrements the contents of a block and stores the result in the internal data
         * register.
         */
        PICC_CMD_MF_DECREMENT(0xC0),
        /**
         * Increments the contents of a block and stores the result in the internal data
         * register.
         */
        PICC_CMD_MF_INCREMENT(0xC1),
        /**
         * Reads the contents of a block into the internal data register.
         */
        PICC_CMD_MF_RESTORE(0xC2),
        /**
         * Writes the contents of the internal data register to a block.
         */
        PICC_CMD_MF_TRANSFER(0xB0),
        // The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
        // The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
        /**
         * Writes one 4 byte page to the PICC.
         */
        PICC_CMD_UL_WRITE(0xA2);

        public final byte value;

        private PICCCommand(int val) {
            value = (byte) val;
        }
    }

    /**
     * PICC types we can detect
     */
    public static enum PICCType {
        /**
         * Unknown PICC type
         */
        PICC_TYPE_UNKNOWN,
        /**
         * PICC compliant with ISO/IEC 14443-4
         */
        PICC_TYPE_ISO_14443_4,
        /**
         * PICC compliant with ISO/IEC 18092 (NFC)
         */
        PICC_TYPE_ISO_18092,
        /**
         * MIFARE Classic protocol, 320 bytes
         */
        PICC_TYPE_MIFARE_MINI,
        /**
         * MIFARE Classic protocol, 1KB
         */
        PICC_TYPE_MIFARE_1K,
        /**
         * MIFARE Classic protocol, 4KB
         */
        PICC_TYPE_MIFARE_4K,
        /**
         * MIFARE Ultralight or Ultralight C
         */
        PICC_TYPE_MIFARE_UL,
        /**
         * MIFARE Plus
         */
        PICC_TYPE_MIFARE_PLUS,
        /**
         * MIFARE DESFire
         */
        PICC_TYPE_MIFARE_DESFIRE,
        /**
         * Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure
         */
        PICC_TYPE_TNP3XXX,
        /**
         * SAK indicates UID is not complete.
         */
        PICC_TYPE_NOT_COMPLETE
    };

    /**
     * Return codes from the functions in this class
     */
    public static enum StatusCode {
        /**
         * Success
         */
        STATUS_OK,
        /**
         * Error in communication
         */
        STATUS_ERROR,
        /**
         * Collision detected
         */
        STATUS_COLLISION,
        /**
         * Timeout in communication
         */
        STATUS_TIMEOUT,
        /**
         * A buffer is not big enough.
         */
        STATUS_NO_ROOM,
        /**
         * Internal error in the code. Should not happen ;-)
         */
        STATUS_INTERNAL_ERROR,
        /**
         * Invalid argument.
         */
        STATUS_INVALID,
        /**
         * The CRC_A does not match
         */
        STATUS_CRC_WRONG,
        /**
         * A MIFARE PICC responded with NAK.
         */
        STATUS_MIFARE_NACK
    };

    /**
     * A record used for passing the UID of a PICC.
     */
    public record UID(
            /**
             * Number of bytes in the UID. 4, 7, or 10.
             */
            byte size,
            /**
             * An array of length {@code size}, containing the bytes of the UID
             */
            byte[] uidByte,
            /**
             * The SAK (Select acknowledge) byte returned from the PICC after successful
             * selection.
             */
            byte sak) {}

    /**
     * Used by readCardSerial.
     */
    public UID uid;

    private final SPI spi;
    private final Optional<DigitalOutput> resetPowerDown;

    /**
     * Create a new object for interfacing with an RC522 RFID module.
     * 
     * @param chipSelectPort    {@link SPI.Port} that the RC522 chip select pin is
     *                          connected to
     * @param resetPowerDownPin DIO port that the RC522 reset and power down pin is
     *                          connected to
     * @see SPI
     * @see DigitalOutput
     */
    public RC522(SPI.Port chipSelectPort, int resetPowerDownPin) {
        spi = new SPI(chipSelectPort);
        resetPowerDown = Optional.of(new DigitalOutput(resetPowerDownPin));
        spi.setClockRate(SPI_CLOCK);
        spi.setMode(SPI.Mode.kMode0);
    }

    /**
     * Create a new object for interfacing with an RC522 RFID module which isn't
     * connected to the reset and power down pin. In this case, only soft reset will
     * be used in {@link init}.
     * 
     * @param chipSelectPort {@link SPI.Port} that the RC522 chip select pin is
     *                       connected to.
     * @see SPI
     */
    public RC522(SPI.Port chipSelectPort) {
        spi = new SPI(chipSelectPort);
        resetPowerDown = Optional.empty();
    }

    /**
     * Writes a byte to the specified register in the RC522 chip. The interface is
     * described in the datasheet section 8.1.2.
     * 
     * @param reg   The register to write to
     * @param value The value to write
     */
    public void writeRegister(PCDRegister reg, byte value) {
        spi.write(new byte[] { reg.value, value }, 2); // MSB == 0 is for writing; see datasheet section 8.1.2.3
    }

    /**
     * Writes a byte to the specified register in the RC522 chip. The interface is
     * described in the datasheet section 8.1.2.
     * 
     * @param reg   The register to write to
     * @param value The value to write, will be cast to byte
     */
    public void writeRegister(PCDRegister reg, int value) {
        writeRegister(reg, (byte) value);
    }

    /**
     * Writes a number of bytes to the specified register in the RC522 chip. The
     * interface is described in the datasheet section 8.1.2.
     * 
     * @param reg    The register to write to
     * @param values The values to write; values.limit is how many bytes are written
     */
    public void writeRegister(PCDRegister reg, ByteBuffer values) {
        spi.write(new byte[] { reg.value }, 1);
        spi.write(values, values.limit());
    }

    /**
     * Sends a command.
     * 
     * @param command The command to send
     */
    public void command(PCDCommand command) {
        writeRegister(PCDRegister.CommandReg, command.value);
    }

    /**
     * Reads a byte from the specified register in the RC522 chip. The interface is
     * described in the datasheet section 8.1.2.
     * 
     * @param reg The register to read from
     * @return One byte read from the register
     */
    public byte readRegister(PCDRegister reg) {
        byte[] result = new byte[1];
        spi.write(new byte[] { (byte) (0x80 | reg.value) }, 1); // MSB == 1 is for reading; see datasheet section 8.1.2.3
        spi.transaction(new byte[] { 0 }, result, 1);
        return result[0];
    }

    /**
     * Reads a number of bytes from the specified register in the RC522 chip. The
     * interface is described in the datasheet section
     * 
     * @param reg   The register to read from
     * @param count The number of bytes to read
     * @return A {@link ByteBuffer} containing {@code count} bytes read from the
     *         chip
     */
    public ByteBuffer readRegister(PCDRegister reg, int count) {
        if (count == 0) return ByteBuffer.allocate(0);

        ByteBuffer address = ByteBuffer.allocate(count);
        for (int i = 0; i < count - 1; i++)
            address.put((byte) (0x80 | reg.value));
        address.put((byte) 0);
        ByteBuffer result = ByteBuffer.allocate(count);
        spi.write(new byte[] { (byte) (0x80 | reg.value) }, 1);
        spi.transaction(address, result, count);

        return result.flip();
    }

    /**
     * Sets the bits given in a bitmask in the specified register.
     * 
     * @param reg  The register to update
     * @param mask The bits to set
     */
    public void setRegisterBitmask(PCDRegister reg, byte mask) {
        writeRegister(reg, readRegister(reg) | mask);
    }

    /**
     * Sets the bits given in a bitmask in the specified register.
     * 
     * @param reg  The register to update
     * @param mask The bits to set; cast to byte
     */
    public void setRegisterBitmask(PCDRegister reg, int mask) {
        setRegisterBitmask(reg, (byte) mask);
    }

    /**
     * Clears the bits given in a bitmask in the specified register.
     * 
     * @param reg  The register to update
     * @param mask The bits to clear
     */
    public void clearRegisterBitmask(PCDRegister reg, byte mask) {
        writeRegister(reg, readRegister(reg) & ~mask);
    }

    /**
     * Clears the bits given in a bitmask in the specified register.
     * 
     * @param reg  The register to update
     * @param mask The bits to clear; cast to byte
     */
    public void clearRegisterBitmask(PCDRegister reg, int mask) {
        clearRegisterBitmask(reg, (byte) mask);
    }

    /**
     * Use the CRC coprocessor in the RC522 to calculate a CRC_A.
     * 
     * @param data The data to transfer to the FIFO for CRC calculation
     * @return Optional containing the result, low byte first, if successful, or
     *         empty if error (CRC calculation timeout after 90ms)
     */
    public Optional<ByteBuffer> calculateCRC(ByteBuffer data) {
        command(PCDCommand.PCD_Idle); // Stop any active command
        writeRegister(PCDRegister.DivIrqReg, 0x04); // Clear the CRCIRq interrupt request bit
        writeRegister(PCDRegister.FIFOLevelReg, 0x80); // FlushBuffer = 1, FIFO initialization
        writeRegister(PCDRegister.FIFODataReg, data); // Write data to the FIFO
        command(PCDCommand.PCD_CalcCRC); // Start the calculation

        ByteBuffer result = ByteBuffer.allocate(2);

        /*
         * Wait for the CRC calculation to complete. Check for the register to indicate
         * that the CRC calculation is complete in a loop. If the calculation is not
         * indicated as complete in ~90ms, then time out the operation.
         */
        final double deadline = Timer.getFPGATimestamp() + 0.089;

        do {
            // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
            byte n = readRegister(PCDRegister.DivIrqReg);
            if ((n & 0x04) > 0) { // CRCIrq bit set - caulculation done
                command(PCDCommand.PCD_Idle); // stop calculating CRC for new content in the FIFO
                return Optional.of(result.put(readRegister(PCDRegister.CRCResultRegL))
                        .put(readRegister(PCDRegister.CRCResultRegH)).flip());
            }
            Timer.delay(0.0005); // check every 500 microseconds
        } while (Timer.getFPGATimestamp() < deadline);

        return Optional.empty();
    }

    /**
     * Initializes the RC522 chip. If a reset and power down connection is provided,
     * preforms a hard reset; otherwise, performs a soft reset.
     */
    public void init() {
        boolean hardReset = false;

        if (resetPowerDown.isPresent()) {
            // Check if chip is in power down mode
            if (!resetPowerDown.get().get()) {
                resetPowerDown.get().set(false); // make sure we have a clean LOW state
                Timer.delay(0.000002); // 8.8.1 Reset timing requirements says about 100ns. Let us be generous: 2μs
                resetPowerDown.get().set(true); // Exit power down mode. This triggers a hard reset.
                // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37.74μs. Let us be generous: 50ms.
                Timer.delay(0.050);
                hardReset = true;
            }
        }

        if (!hardReset) reset();

        // Reset baud rates
        writeRegister(PCDRegister.TxModeReg, 0x00);
        writeRegister(PCDRegister.RxModeReg, 0x00);
        // Reset ModWidthReg
        writeRegister(PCDRegister.ModWidthReg, 0x26);

        /*
         * When communicating with a PICC we need a timeout if something goes wrong.
         * f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler =
         * [TPrescaler_Hi:TPrescaler_Lo]. TPrescaler_Hi are the four low bits in
         * TModeReg. TPrescaler_Lo is TPrescalerReg.
         */
        writeRegister(PCDRegister.TModeReg, 0x80); // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
        writeRegister(PCDRegister.TPrescalerReg, 0xA9); // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
        writeRegister(PCDRegister.TReloadRegH, 0x03); // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
        writeRegister(PCDRegister.TReloadRegL, 0xE8);

        writeRegister(PCDRegister.TxASKReg, 0x40); // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
        writeRegister(PCDRegister.ModeReg, 0x3D); // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
        antennaOn(); // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
    }

    /**
     * Performs a soft reset of the RC522 chip and waits for it to be ready again.
     */
    public void reset() {
        command(PCDCommand.PCD_SoftReset);
        /*
         * The datasheet does not mention how long the SoftReset command takes to
         * complete. But the MFRC522 might have been in soft power-down mode (triggered
         * by bit 4 of CommandReg) Section 8.8.2 in the datasheet says the oscillator
         * start-up time is the start up time of the crystal + 37.74μs. Let us be
         * generous: 50ms.
         */
        int count = 0;
        do {
            Timer.delay(0.050); // Wait for the PowerDown bit in CommandReg to be cleared, max 3 waits of 50ms
        } while ((readRegister(PCDRegister.CommandReg) & (1 << 4)) > 0 && (++count) < 3);
    }

    /**
     * Turns the antenna on by enabling pins TX1 and TX2. After a reset these pins
     * are disabled.
     */
    public void antennaOn() {
        setRegisterBitmask(PCDRegister.TxControlReg, 0x03);
    }

    /**
     * Turns the antenna off by disabling pins TX1 and TX2.
     */
    public void antennaOff() {
        clearRegisterBitmask(PCDRegister.TxControlReg, 0x03);
    }

    /**
     * Get the current RC522 Receiver Gain (RxGain[2:0]) value. See 9.3.3.6 / table
     * 98 in the datasheet. NOTE: Return value scrubbed with (0x07{@literal <<}
     * 4)=01110000b as RCFfgReg may use reserved bits.
     * 
     * @return Value of the RxGain, scrubbed to the 3 bits used.
     */
    public byte getAntennaGain() {
        return (byte) (readRegister(PCDRegister.RFCfgReg) & (0x07 << 4));
    }

    /**
     * Set the RC522 Receiver Gain (RxGain) to value specified by given mask. See
     * 9.3.3.6 / table 98 in the datasheet. NOTE: Given mask is scrubbed with
     * (0x07{@literal <<} 4)=01110000b as RCFfgReg may use reserved bits.
     * 
     * @param mask The mask to use; bits 7, 6, and 5 represent the antenna gain
     */
    public void setAntennaGain(byte mask) {
        if (getAntennaGain() != mask) {
            clearRegisterBitmask(PCDRegister.RFCfgReg, 0x07 << 4);
            setRegisterBitmask(PCDRegister.RFCfgReg, mask & (0x07 << 4));
        }
    }

    /**
     * Performs a self-test of the RC522. See 16.1.1 in the datasheet.
     * 
     * @return Whether or not the test passed. Or false if no firmware reference is
     *         available.
     */
    public boolean performSelfTest() {
        // This follows directly the steps outlined in 16.1.1
        // 1. Perform a soft reset.
        reset();

        // 2. Clear the internal buffer by writing 25 null bytes
        ByteBuffer zeros = ByteBuffer.allocate(25);
        for (int i = 0; i < 25; i++)
            zeros.put((byte) 0x00);
        zeros.flip();
        writeRegister(PCDRegister.FIFOLevelReg, 0x80); // flush the FIFO buffer
        writeRegister(PCDRegister.FIFODataReg, zeros); // write 25 null bytes to FIFO
        command(PCDCommand.PCD_Mem); // transfer to internal buffer

        // 3. Enable self-test
        writeRegister(PCDRegister.AutoTestReg, 0x09);

        // 4. Write a null byte to FIFO buffer
        writeRegister(PCDRegister.FIFODataReg, 0x00);

        // 5. Start self-test by issuing the CalcCRC command
        command(PCDCommand.PCD_CalcCRC);

        // 6. Wait for self-test to complete
        for (int i = 0; i < 0xFF; i++) {
            /*
             * The datasheet does not specify exact completion condition except that FIFO
             * buffer should contain 64 bytes. While selftest is initiated by CalcCRC
             * command it behaves differently from normal CRC computation, so one can't
             * reliably use DivIrqReg to check for completion. It is reported that some
             * devices does not trigger CRCIRq flag during selftest.
             */
            if (readRegister(PCDRegister.FIFOLevelReg) >= 64) break;
        }
        command(PCDCommand.PCD_Idle);

        // 7. Read out result 64 bytes from the FIFO buffer.
        ByteBuffer result = readRegister(PCDRegister.FIFODataReg, 64);

        // Auto self-test done. Reset AutoTestReg to 0, required for normal operation.
        writeRegister(PCDRegister.AutoTestReg, 0x00);

        // Determine firmware version (see section 9.3.4.8 in datasheet)
        byte version = readRegister(PCDRegister.VersionReg);

        final ByteBuffer reference = switch (version) {
            case (byte) 0x88 -> FM17522_FIRMWARE_REFERENCE;
            case (byte) 0x90 -> RC522_FIRMWARE_REFERENCE_V0_0;
            case (byte) 0x91 -> RC522_FIRMWARE_REFERENCE_V1_0;
            case (byte) 0x92 -> RC522_FIRMWARE_REFERENCE_V2_0;
            default -> ByteBuffer.allocate(0); // unknown version, abort test
        };
        if (reference.capacity() == 0) return false;

        if (!reference.equals(result)) return false;

        /*
         * 8. Perform a re-init, because PCD does not work after test. Reset does not
         * work as expected. "Auto self-test done" does not work as expected.
         */
        init();

        return true;
    }
}