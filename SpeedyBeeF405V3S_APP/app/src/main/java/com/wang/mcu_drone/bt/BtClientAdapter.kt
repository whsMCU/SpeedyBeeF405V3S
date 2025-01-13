package com.wang.mcu_drone.bt

import android.bluetooth.BluetoothDevice
import android.content.Context
import android.view.ViewGroup
import android.widget.TextView
import com.chad.library.adapter4.BaseQuickAdapter
import com.chad.library.adapter4.viewholder.QuickViewHolder
import com.wang.mcu_drone.R
import com.wang.mcu_drone.utils.getDeviceBond
import com.wang.mcu_drone.utils.getDeviceName


/**
 * FileName: BtClientAdapter
 * Author: JiaoCan
 * Date: 2024/5/8 10:44
 */

class BtClientAdapter : BaseQuickAdapter<BluetoothDevice, QuickViewHolder>() {
    override fun onBindViewHolder(holder: QuickViewHolder, position: Int, item: BluetoothDevice?) {
        if (item == null) return
        val sb = context.getDeviceName(item) + "\n" + item.address +
                if (context.getDeviceBond(item) == BluetoothDevice.BOND_BONDED) " (페어링됨) " else " (페어링되지 않음) "
        holder.getView<TextView>(R.id.content).text = sb
    }

    override fun onCreateViewHolder(context: Context, parent: ViewGroup, viewType: Int): QuickViewHolder {
        return QuickViewHolder(R.layout.item_ble, parent)
    }
}

