package com.wang.mcu_drone

import android.app.Application
import android.content.Context

class MyApplication : Application() {

    override fun onCreate() {
        super.onCreate()
        instance = this
    }

    companion object {

        private lateinit var instance: MyApplication

        fun getContext(): Context {
            return instance.applicationContext
        }

    }
}
