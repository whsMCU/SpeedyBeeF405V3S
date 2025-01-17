package com.wang.mcu_drone.protocol

import android.health.connect.datatypes.units.Temperature
import java.util.LinkedList

class MspProtocol {

    enum class command(val value:Int){
        TELEMERY_PID_SAVE(56),
        MSP_IDENT(100),
        MSP_STATUS(101),
        MSP_RAW_IMU(102),
        MSP_MOTOR(104),
        MSP_RC(105),
        MSP_RAW_GPS(106),
        MSP_ATTITUDE(108),
        MSP_ALTITUDE(109),
        MSP_PID(112),
        MSP_MISC(114),
        MSP_PIDNAMES(117),

        MSP_RC_RAW(150), //out message
        MSP_ARM(151),
        MSP_DISARM(152),
        MSP_SET_RAW_RC(200),
        MSP_SET_RAW_GPS(201),
        MSP_SET_PID(202),
        MSP_ACC_CALIBRATION(205),
        MSP_MAG_CALIBRATION(206),
        MSP_SET_MISC(207),
        MSP_SET_MOTOR(214),

        MSP_EEPROM_WRITE(250),
        MSP_DEBUG(254)
    }

    enum class parser {
        IDLE,
        HEADER_START,
        HEADER_M,
        HEADER_ARROW,
        HEADER_SIZE,
        HEADER_CMD,
        HEADER_ERR
    }

    var c_state: parser = parser.IDLE

    var INBUF_SIZE: Int = 256
    var inBuf: ByteArray = ByteArray(1024)
    var checksum: Byte = 0
    var indRX: Byte = 0
    var cmdMSP: Byte = 0
    var offset: Int = 0
    var dataSize: Int = 0

    private val MSP_HEADER: String = "\$M<"

    var payload: List<Char>? = null

    var PIDITEMS: Int = 3

    var error_count: Int = 0
    var arry: IntArray = intArrayOf(0)


    var armedTime: Int = 0
    var hours: Int = 0
    var minutes: Int = 0
    var seconds: Int = 0
    var mode: Int = 0
    var ARMED: Int = 0
    var HEADFREE_MODE: Int = 0
    var ANGLE_MODE: Int = 0
    var HORIZON_MODE: Int = 0
    var ACRO_MODE: Int = 0
    var cycleTime: Int = 0
    var error: Int = 0

    var Roll_In = floatArrayOf(0f,0f,0f)
    var Roll_Out = floatArrayOf(0f,0f,0f)

    var Pitch_In = floatArrayOf(0f,0f,0f)
    var Pitch_Out = floatArrayOf(0f,0f,0f)

    var Yaw_In = floatArrayOf(0f,0f,0f)
    var Yaw_Out = floatArrayOf(0f,0f,0f)

    var Yaw_Heading = floatArrayOf(0f,0f,0f)
    var Yaw_Rate = floatArrayOf(0f,0f,0f)

    fun requestMSP(msp: Int): List<Byte>? {
        return requestMSP(msp, null)
    }

    //send multiple msp without payload
    fun requestMSP(msps: IntArray): List<Byte> {
        val s: MutableList<Byte> = LinkedList()
        for (m in msps) {
            s.addAll(requestMSP(m, null)!!)
        }
        return s
    }

    fun requestMSP(msp: Int, payload: Array<Char>?): List<Byte>? {
        if (msp < 0) {
            return null
        }
        val bf: MutableList<Byte> = LinkedList()
        for (c in MSP_HEADER.toByteArray()) {
            bf.add(c)
        }
        var checksum: Byte = 0
        val pl_size = (payload?.size ?: 0).toByte()
        bf.add(pl_size)
        checksum = (checksum.toInt() xor (pl_size.toInt() and 0xFF)).toByte()

        bf.add((msp and 0xFF).toByte())
        checksum = (checksum.toInt() xor (msp and 0xFF)).toByte()

        if (payload != null) {
            for (c in payload) {
                bf.add((c.code and 0xFF).toByte())
                checksum = (checksum.toInt() xor (c.code and 0xFF)).toByte()
            }
        }
        bf.add(checksum)
        return (bf)
    }

    fun sendRequestMSP(msp: List<Byte>): ByteArray {
        val arr = ByteArray(msp.size)
        var i = 0
        for (b in msp) {
            arr[i++] = b
        }
        return arr
    }

    fun SerialCom(i: ByteArray) {
        for (c in i) {
            if (c_state == parser.IDLE) {
                c_state = if ((c == '$'.code.toByte())) parser.HEADER_START else parser.IDLE
                //Log.d("msp", "c_state : " + c_state);
            } else if (c_state == parser.HEADER_START) {
                c_state = if ((c == 'M'.code.toByte())) parser.HEADER_M else parser.IDLE
                //Log.d("msp", "c_state : " + c_state);
            } else if (c_state == parser.HEADER_M) {
                c_state = if ((c == '>'.code.toByte())) parser.HEADER_ARROW else parser.IDLE
                //Log.d("msp", "c_state : " + c_state);
            } else if (c_state == parser.HEADER_ARROW) {
                if (c > INBUF_SIZE) {  // now we are expecting the payload size
                    c_state = parser.IDLE
                    //Log.d("msp", "c_state : " + c_state);
                    continue
                }
                dataSize = c.toInt()
                offset = 0
                indRX = 0
                checksum = 0
                checksum = (checksum.toInt() xor c.toInt()).toByte()
                c_state = parser.HEADER_SIZE
                //Log.d("msp", "c_state : " + c_state);
            } else if (c_state == parser.HEADER_SIZE) {
                cmdMSP = c
                checksum = (checksum.toInt() xor c.toInt()).toByte()
                c_state = parser.HEADER_CMD
                //Log.d("msp", "cmdMSP : " + cmdMSP);
                //Log.d("msp", "c_state : " + c_state);
            } else if (c_state == parser.HEADER_CMD && offset < dataSize) {
                checksum = (checksum.toInt() xor c.toInt()).toByte()
                inBuf[offset++] = c
                //Log.d("msp", "offset : " + offset);
                //Log.d("msp", "dataSize : " + dataSize);
                //Log.d("msp", "inBuf : " + inBuf[offset-1]);
                //Log.d("msp", "checksum : " + checksum);
            } else if (c_state == parser.HEADER_CMD && offset >= dataSize) {
                //Log.d("msp", "-----------------------------");
                //Log.d("msp", "checksum : " + checksum);
                //Log.d("msp", "c : " + c);
                if (checksum == c) {
                    //Log.d("msp1", "checksum OK!");
                    evaluateCommand()
                } else {
                    error_count++
                    //Log.d("error", "Test"+error_count);
                }
                c_state = parser.IDLE
            }
        }
    }

    private fun evaluateCommand() {
        when (cmdMSP.toInt()) {
            command.MSP_PID ->
                //Log.d("msp_pid", "OK");
                //Log.d("msp_pid", "OK"+byteP[0]);
            {
                Roll_In[0] = read32().toFloat()
                Roll_In[1] = read32().toFloat()
                Roll_In[2] = read32().toFloat()

                Roll_Out[0] = read32().toFloat()
                Roll_Out[1] = read32().toFloat()
                Roll_Out[2] = read32().toFloat()

                Pitch_In[0] = read32().toFloat()
                Pitch_In[1] = read32().toFloat()
                Pitch_In[2] = read32().toFloat()

                Pitch_Out[0] = read32().toFloat()
                Pitch_Out[1] = read32().toFloat()
                Pitch_Out[2] = read32().toFloat()

                Yaw_In[0] = read32().toFloat()
                Yaw_In[1] = read32().toFloat()
                Yaw_In[2] = read32().toFloat()

                Yaw_Out[0] = read32().toFloat()
                Yaw_Out[1] = read32().toFloat()
                Yaw_Out[2] = read32().toFloat()

                Yaw_Heading[0] = read32().toFloat()
                Yaw_Heading[1] = read32().toFloat()
                Yaw_Heading[2] = read32().toFloat()

                Yaw_Rate[0] = read32().toFloat()
                Yaw_Rate[1] = read32().toFloat()
                Yaw_Rate[2] = read32().toFloat()

            }
        }
    }

    fun read32(): Int {
        return (inBuf[indRX++.toInt()].toInt() and 0xff) + ((inBuf[indRX++.toInt()].toInt() and 0xff) shl 8) + ((inBuf[indRX++.toInt()].toInt() and 0xff) shl 16) + ((inBuf[indRX++.toInt()].toInt() and 0xff) shl 24)
    }

    fun read16(): Int {
        return (inBuf[indRX++.toInt()].toInt() and 0xff) + ((inBuf[indRX++.toInt()]).toInt() shl 8)
    }

    fun read8(): Int {
        return (inBuf[indRX++.toInt()].toInt() and 0xff)
    }
}