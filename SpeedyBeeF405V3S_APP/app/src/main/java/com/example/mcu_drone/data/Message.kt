package com.example.mcu_drone.data

data class Message(val text: String, val device: Device?=null, val isMine: Boolean)