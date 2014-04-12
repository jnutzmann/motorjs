if (typeof(controller) === "undefined") { window.controller = {}; }

controller.pi = function () {
    this.reference = 0;
	this.measured = 0;
	this.accumulator = 0;
	this.dt = 0;
	this.kp = 0;
	this.ki = 0;
	this.outMax = 0;
	this.outMin = 0;

    this.control = function() {
        var error = this.reference - this.measured;
        this.accumulator = this.limit( (error * this.ki * this.dt + this.accumulator), this.outMax, this.outMin);
        return this.limit( error * this.kp + this.accumulator, this.outMax, this.outMin);
    }

    this.limit = function (val, upper, lower) {
        if ( val > upper ) return upper;
        if ( val < lower ) return lower;
        return val;
    }

    
    return this;
}

controller.foc = function () {

    this.Vbus = 0;
    this.Ia = 0;
    this.Ib = 0;
    this.Ic = 0;
    this.Ialpha = 0;
    this.Ibeta = 0;
    this.Id = 0;
    this.Iq = 0;
    this.theta = 0;
    this.sinTheta = 0;
    this.cosTheta = 0;
    this.Vd = 0;
    this.Vq = 0;
    this.Valpha = 0;
    this.Vbeta = 0;
    this.Va = 0;
    this.Vb = 0;
    this.Vc = 0;

    this.V1 = 0;
    this.V2 = 0;
    this.V3 = 0;
    this.Vk = 0;
    
    this.clarke = function( ) {
        this.Ialpha = this.Ia;
        this.Ibeta  = ((1/Math.sqrt(3)) * this.Ia) + ((2/Math.sqrt(3)) * this.Ib);
    }
    
    this.sine_cosine = function( control ) {
        var rad = this.theta * Math.PI / 180;
    
        this.sinTheta = Math.sin(rad);
        this.cosTheta = Math.cos(rad);
    }
    
    this.park = function ( )
    {
        this.Id =  this.Ialpha * this.cosTheta + this.Ibeta * this.sinTheta;
        this.Iq = -this.Ialpha * this.sinTheta + this.Ibeta * this.cosTheta;
    }
    
    this.inverse_park = function ( )
    {
        this.Valpha = this.Vd * this.cosTheta - this.Vq * this.sinTheta;
        this.Vbeta =  this.Vd * this.sinTheta + this.Vq * this.cosTheta;
    }
    
    this.inverse_clarke = function ( )
    {
        this.Va = this.Valpha;
        this.Vb = -this.Valpha * 0.5 + Math.sqrt(3)/2 * this.Vbeta;
        this.Vc = -this.Valpha * 0.5 - Math.sqrt(3)/2 * this.Vbeta;
    }
    
    this.svm = function ( )
    {
    
        this.Vk = ( ( Math.max(this.Va, this.Vb, this.Vc) + Math.min(this.Va, this.Vb, this.Vc) ) * 0.5 );
    
        var halfVbus = 0.5 * this.Vbus;
    
        this.V1 = (this.Va - this.Vk) + halfVbus;
        this.V2 = (this.Vb - this.Vk) + halfVbus;
        this.V3 = (this.Vc - this.Vk) + halfVbus;
    }

    return this;
}

controller.fullcurrent = function () {

    this.foc = new controller.foc();
    this.piId = new controller.pi();
    this.piIq = new controller.pi();

    //d1k_fram_Read(FRAM_ADDRESS_MOTOR_PHASING_SWAPPED,&swapAAndBPhases,sizeof(swapAAndBPhases));

	this.InitControlLoopConstants();

    this.cp = {}
	this.cp.drivemode = DRIVEMODE_INVALID;
	this.cp.pendingDrivemode = DRIVEMODE_INVALID;

	SetDrivemode( DRIVEMODE_DISABLED );

	SetVDMax( 160.0 );
	SetVDMin( -160.0 );
	SetVQMax( 160.0 );
	SetVQMin( -160.0 );

	SetIQRef( 0.0 );
	SetIDRef( 0.0 );

	this.piIq.dt = DTI;
	this.piId.dt = DTI;

	pwm_Init(PWM_FREQUENCY_HZ);
	pwm_SetPWMOutputEnable( DISABLE, DISABLE, DISABLE );



    this.InitControlLoopConstants = function() {
//        float k;
//
//        d1k_fram_Read(FRAM_ADDRESS_KP,&k,sizeof(k));
//        piIq.kp = k; piId.kp = k;
//        d1k_fram_Read(FRAM_ADDRESS_KI,&k,sizeof(k));
//        piIq.ki = k; piId.ki = k;
//        d1k_fram_Read(FRAM_ADDRESS_AVERAGINGPER,&k,sizeof(k));
//        rolloffAveragingPeriod = k;
    }

    this.control_loop = function () {

        static uint32 waitCount = 0;
        static uint32 settleTime = 10000;
        static uint32 currentErrorFilter = 0;

        waitCount = (waitCount + 1)
                    % ( (PWM_FREQUENCY_HZ * 2) / CONTROL_LOOP_FREQUENCY_HZ );

        if ( waitCount == 0 )
        {
            GPIO_SetBits(GPIOB,GPIO_Pin_15);

            // Check to see if we have a new drivemode.  Set the gates accordingly.
            checkForStateChange();

            // Update the analog measurements.
            UpdateAnalogChannels();

            PolePosition_UpdateVelocityCalculation();

            // Check for current faults.  Ignore them for the first settle time.
            if (settleTime > 0) settleTime--;

            if ( diag_GetHardwareOvercurrent() )
            {
                if ( settleTime == 0 )
                {
                    diag_ThrowError(ERROR_CURRENT_HARDWARE_FAULT);
                }
            }

            if ( analogReadings.Ia > SOFTWARE_CURRENT_FAULT_SOFT
              || analogReadings.Ia < -SOFTWARE_CURRENT_FAULT_SOFT
              || analogReadings.Ib > SOFTWARE_CURRENT_FAULT_SOFT
              || analogReadings.Ib < -SOFTWARE_CURRENT_FAULT_SOFT
              || analogReadings.Ic > SOFTWARE_CURRENT_FAULT_SOFT
              || analogReadings.Ic < -SOFTWARE_CURRENT_FAULT_SOFT )
            {
                if (settleTime == 0)
                {
                    if ( analogReadings.Ia > SOFTWARE_CURRENT_FAULT_HARD
                          || analogReadings.Ia < -SOFTWARE_CURRENT_FAULT_HARD
                          || analogReadings.Ib > SOFTWARE_CURRENT_FAULT_HARD
                          || analogReadings.Ib < -SOFTWARE_CURRENT_FAULT_HARD
                          || analogReadings.Ic > SOFTWARE_CURRENT_FAULT_HARD
                          || analogReadings.Ic < -SOFTWARE_CURRENT_FAULT_HARD )
                    {
                        diag_ThrowError(ERROR_CURRENT_SOFTWARE_FAULT);
                    }
                    else
                    {
                        currentErrorFilter++;

                        if( currentErrorFilter >= SOFTWARE_CURRENT_FAULT_COUNT )
                        {
                            diag_ThrowError(ERROR_CURRENT_SOFTWARE_FAULT);
                        }
                    }
                }
            }
            else if ( currentErrorFilter > 0 )
            {
                currentErrorFilter--;
            }

            // Check to make sure the motor is present.  Otherwise, throw an error.
            (void) motor_CheckIfPresent( );

            // If there are any errors present, immediately float the gates and put the
            // controller in the disable drivemode.
            if ( diag_GetErrorBitmask() != 0x0 )
            {
                pwm_SetPWMOutputEnable(DISABLE,DISABLE,DISABLE);
                SetDrivemode(DRIVEMODE_DISABLED);
                return;
            }

            foc.theta = MotorPosition_GetElectricalPosition() - 90.0f;

            static uint32 startupWaitTime = CONTROL_LOOP_FREQUENCY_HZ * 3;

            if ( startupWaitTime > 0 )
            {
                startupWaitTime--;
                return;
            }

            static uint32 logCnt = 0;

            if ( cp.drivemode == DRIVEMODE_FORWARD || cp.drivemode == DRIVEMODE_REVERSE )
            {
                FOCTorqueControl();
    /*
                static float lastRef = 0.0f;

                if (fabs(cp.iqRef-lastRef) > 10.0f)
                {
                    hsLogEnabled = true;
                    hsLogIndex = 0;
                }

                lastRef = cp.iqRef;

                if ( hsLogEnabled & hsLogIndex >= (HS_LOG_DATA_COUNT-1) )
                {
                    hsLogEnabled = false;
                    hsLogIndex = 0;
                }
    */
                if (hsLogEnabled)
                {
                    logCnt = (logCnt + 1) % 5;

                    if ( logCnt == 0 )
                    {
                        hsLog[hsLogIndex].id = foc.Id;
                        hsLog[hsLogIndex].iq = foc.Iq;
                        hsLog[hsLogIndex].vd = foc.Vd;
                        hsLog[hsLogIndex].vq = foc.Vq;
                        hsLog[hsLogIndex].vbus = analogReadings.Vbus;
                        hsLog[hsLogIndex].motorPos = foc.theta;

                        hsLogIndex = (hsLogIndex+1)%HS_LOG_DATA_COUNT;
                    }
                }
            }
            else
            {

                float n = analogReadings.Va + analogReadings.Vb + analogReadings.Vc;

                n *= (1.0f/3.0f);

                float van, vbn;

                if ( swapAAndBPhases )
                {
                    van = analogReadings.Vb - n;
                    vbn = analogReadings.Va - n;
                }
                else
                {
                    van = analogReadings.Va - n;
                    vbn = analogReadings.Vb - n;
                }

                float valpha = van;
                float vbeta  = ((1.0f/sqrtf(3.0f)) * van)
                             + ((2.0f/sqrtf(3.0f)) * vbn);

                float cosTheta, sinTheta;



                arm_sin_cos_f32(WrapAngle(foc.theta),&sinTheta,&cosTheta);

                float vd =  valpha * cosTheta + vbeta * sinTheta;
                float vq = -valpha * sinTheta + vbeta * cosTheta;

                foc.Vd = vd;
                foc.Vq = vq;

                piId.accumulator = vd;
                piIq.accumulator = vq;
            }

            GPIO_ResetBits(GPIOB,GPIO_Pin_15);
        }

    }


    this.foc_torque_control = function ( ) {

        static bool brakingEnabled = false;

        float mechVelocity = MotorPosition_GetMechanicalVelocity();

        foc.theta = WrapAngle(foc.theta);

        if ( swapAAndBPhases )
        {
            foc.Ia =  -analogReadings.Ib;
            foc.Ib =  -analogReadings.Ia;
        }
        else
        {
            foc.Ia =  -analogReadings.Ia;
            foc.Ib =  -analogReadings.Ib;
        }

        foc.Ic =  -analogReadings.Ic;
        foc.Vbus = analogReadings.Vbus;

        if ( cp.drivemode == DRIVEMODE_FORWARD )
        {
            if (mechVelocity > 10.0f )
            {
                brakingEnabled = true;
            }
            else if ( mechVelocity < 2.0f )
            {
                brakingEnabled = false;
            }

            // If we are braking while going forward, stop braking at 2 RPM
            if (!brakingEnabled && cp.iqRef < 0 )
            {
                cp.iqRef = 0;
            }

            // Filter the input command.
            piIq.reference = cp.iqRef;
        }
        else if ( cp.drivemode == DRIVEMODE_REVERSE )
        {
            if (mechVelocity < -10.0f )
            {
                brakingEnabled = true;
            }
            else if ( mechVelocity > -2.0f )
            {
                brakingEnabled = false;
            }

            // If we are braking while going forward, stop braking at 2 RPM
            if (!brakingEnabled && cp.iqRef < 0 )
            {
                cp.iqRef = 0;
            }

            piIq.reference = cp.iqRef * -1.0f ;
        }
        else
        {
            // This should never happen.
            cp.iqRef = 0;
        }

        piId.reference = cp.idRef;

        FOC_SinCos( &foc );
        FOC_Clarke( &foc );
        FOC_Park  ( &foc );

        piId.measured = foc.Id;
        piIq.measured = foc.Iq;

        piIq.outMax = limitf32( foc.Vbus * VQ_LIMIT_PERCENT, cp.vqMax, -200.0f);
        piIq.outMin = limitf32( -piIq.outMax, 200.0f, cp.vqMin);

        float vqn = PI_Control( &piIq );
        foc.Vq = (foc.Vq * (rolloffAveragingPeriod-1.0f) + vqn) / rolloffAveragingPeriod;

        // Find the limit of Vd such that the length of vs does not exceed vbus*VDQ_LIMIT_PERCENT

        float VsLimit = foc.Vbus * VDQ_LIMIT_PERCENT;

        float vdLimit = sqrtf( (VsLimit * VsLimit) - (foc.Vq * foc.Vq) );

        piId.outMax = limitf32( vdLimit, cp.vdMax, -200.0f);
        piId.outMin = limitf32( -piId.outMax, 200.0f, cp.vdMin);

        float vdn = PI_Control( &piId );
        foc.Vd = (foc.Vd * (rolloffAveragingPeriod-1.0f) + vdn) / rolloffAveragingPeriod;


        FOC_InvPark  ( &foc );
        FOC_InvClarke( &foc );
        FOC_SVM      ( &foc );

        float oneOverVbus = 1/analogReadings.Vbus;

        if ( swapAAndBPhases )
        {
            pwm_SetDutyCycles( foc.V2*oneOverVbus, foc.V1*oneOverVbus, foc.V3*oneOverVbus );
        }
        else
        {
            pwm_SetDutyCycles( foc.V1*oneOverVbus, foc.V2*oneOverVbus, foc.V3*oneOverVbus );
        }

    }


    return this;
}




