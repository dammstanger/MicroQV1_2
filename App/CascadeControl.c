    void GET_EXPRAD(void)                        //?????и▓ик????ии,2??ио????и║б└?и▓ик????ии?a0,0
    {
            EXP_ANGLE.X = (float)(-(Rc_Get.ROLL-1500)/30.0f);       
            EXP_ANGLE.Y = (float)(-(Rc_Get.PITCH-1500)/30.0f);
            EXP_ANGLE.Z = (float)(Rc_Get.YAW);
    //        printf("%f %f\n",MPU6050_ACC_LAST.Y*cos(Q_ANGLE.X/57.3)-MPU6050_ACC_LAST.Z*sin(Q_ANGLE.X/57.3),MPU6050_ACC_LAST.X*cos(-Q_ANGLE.Y/57.3)-MPU6050_ACC_LAST.Z*sin(-Q_ANGLE.Y/57.3));
    //        DIF_ANGLE.X = (ACC_AVG.Y*cos(Q_ANGLE.X/57.3)-ACC_AVG.Z*sin(Q_ANGLE.X/57.3))/500;
    //        DIF_ANGLE.Y = (ACC_AVG.X*cos(-Q_ANGLE.Y/57.3)-ACC_AVG.Z*sin(-Q_ANGLE.Y/57.3)/500);
      DIF_ANGLE.X = EXP_ANGLE.X - Q_ANGLE.X;
      DIF_ANGLE.Y = EXP_ANGLE.Y - Q_ANGLE.Y;
    //        DIF_ANGLE.Z = EXP_ANGLE.Z - GYRO_I[0].Z;
    //        DIF_ANGLE.X = EXP_ANGLE.X - GYRO_I[0].X;
    //        DIF_ANGLE.Y = EXP_ANGLE.Y - GYRO_I[0].Y;
    //        DIF_ANGLE.Z = EXP_ANGLE.Z - GYRO_I[0].Z;
    }

    void CONTROL(void)
    {
            static float thr=0,rool=0,pitch=0,yaw=0;
            static float rool_i=0,pitch_i=0;
            static float rool_dif=0,pitch_dif=0;
            static float rool_speed_dif=0,pitch_speed_dif=0;
            float rool_out,pitch_out;
            uint16_t THROTTLE;

            GET_EXPRAD();
           
            rool         = PID_ROL.P * DIF_ANGLE.X;       
            rool_i += PID_ROL.I * DIF_ANGLE.X * 0.002;
            rool_i = between(rool_i,30,-30);
            rool += rool_i;
            rool += PID_ROL.D * (DIF_ANGLE.X-rool_dif) * 500;
            rool_dif = DIF_ANGLE.X;
      ///////////       
            pitch         = PID_ROL.P * DIF_ANGLE.Y;       
            pitch_i += PID_ROL.I * DIF_ANGLE.Y * 0.002;
            pitch_i = between(pitch,30,-30);
            pitch += pitch_i;               
            pitch += PID_ROL.D * (DIF_ANGLE.Y-pitch_dif) * 500;
            pitch_dif = DIF_ANGLE.Y;
            ///////////
    //       
            rool -= GYRO_AVG.X* Gyro_G;
            rool_out=PID_PIT.P*rool;
            rool_out += PID_PIT.D*(rool- rool_speed_dif)*500;
            rool_speed_dif = rool;
           
            pitch -= GYRO_AVG.Y* Gyro_G;
            pitch_out=PID_PIT.P*pitch;
            pitch_out += PID_PIT.D*(pitch- pitch_speed_dif)*500;
            pitch_speed_dif=pitch;

    //        rool=PID_PIT.I*(rool-GYRO_AVG.X* Gyro_G);
    //        pitch=PID_PIT.I*(pitch-GYRO_AVG.Y* Gyro_G);
           
    //        PID_YAW.dout = 20 * (MPU6050_GYRO_LAST.Z* Gyro_G-(Rc_Get.YAW-1500)/10);
            PID_YAW.dout = 10 * (GYRO_AVG.Z* Gyro_G-(Rc_Get.YAW-1500)/10);

            PID_ROL.OUT = rool_out;
            PID_PIT.OUT = pitch_out;
            PID_YAW.OUT = PID_YAW.dout;
           
    /////////////
    //        GYRO_I[0].Z += EXP_ANGLE.Z/3000;
    //        yaw = -10 * GYRO_I[0].Z;
    //       
    //        yaw -= 3 * GYRO_F.Z;       
            THROTTLE=Rc_Get.THROTTLE;
            if(THROTTLE>1050)
            {
    //                if(THROTTLE>1950)
    //                {
    //                        THROTTLE=1950;
    //                }
                    THROTTLE = THROTTLE/cos(Q_ANGLE.X/57.3)/cos(Q_ANGLE.Y/57.3);       
                   
                    moto1 = THROTTLE - 1000 + (int16_t)PID_ROL.OUT - (int16_t)PID_PIT.OUT - (int16_t)PID_YAW.OUT;
                    moto2 = THROTTLE - 1000 + (int16_t)PID_ROL.OUT + (int16_t)PID_PIT.OUT + (int16_t)PID_YAW.OUT;
                    moto3 = THROTTLE - 1000 - (int16_t)PID_ROL.OUT + (int16_t)PID_PIT.OUT - (int16_t)PID_YAW.OUT;
                    moto4 = THROTTLE - 1000 - (int16_t)PID_ROL.OUT - (int16_t)PID_PIT.OUT + (int16_t)PID_YAW.OUT;
            }
            else
            {
                    moto1 = 0;
                    moto2 = 0;
                    moto3 = 0;
                    moto4 = 0;
            }
            if(Q_ANGLE.X>45||Q_ANGLE.Y>45||Q_ANGLE.X<-45||Q_ANGLE.Y<-45)
            {
                    ARMED=0;
                    LED3_OFF;
            }
    //        printf("moto=%d %d %d %d\n",moto1,moto2,moto3,moto4);
            if(ARMED)        MOTO_PWMRFLASH(moto1,moto2,moto3,moto4);
            else                        MOTO_PWMRFLASH(0,0,0,0);
    }