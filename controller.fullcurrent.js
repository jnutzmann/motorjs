if (typeof(controller) === "undefined") { window.controller = {}; }

controller.fullcurrent = function () {

    this.PWM_FREQUENCY_HZ = 20000;
    this.CONTROL_LOOP_FREQUENCY_HZ = this.PWM_FREQUENCY_HZ;
    this.DTI = 1/this.CONTROL_LOOP_FREQUENCY_HZ;
    
    this.VDQ_LIMIT_PERCENT = 0.54; // this limits to 0.957 duty cycle
    this.VQ_LIMIT_PERCENT = this.VDQ_LIMIT_PERCENT * 0.85;	// This limits vector to 1
    this.VD_LIMIT_PERCENT = this.VDQ_LIMIT_PERCENT * 0.5268;

    this.IQ_LIMIT_A = 140.0;
    this.ID_LIMIT_A = 0.0;

    this.SOFTWARE_CURRENT_FAULT_SOFT = 160.0;
    this.SOFTWARE_CURRENT_FAULT_HARD = 185.0;
    this.SOFTWARE_CURRENT_FAULT_COUNT = 120;

    this.drivemode = {
        DRIVEMODE_FORWARD: 0,
        DRIVEMODE_REVERSE: 1,
        DRIVEMODE_DISABLED: 2,
        DRIVEMODE_INVALID: 3
    }

    this.errors = [];

    this.analog_readings = {
        Ia: 0,
        Ib: 0,
        Ic: 0,
        Va: 0,
        Vb: 0,
        Vc: 0,
        Vbus: 130,
        Vgate: 15
    }

    this.cp = {
        drivemode: this.drivemode.DRIVEMODE_INVALID,
        pendingDrivemode: this.drivemode.DRIVEMODE_INVALID,
        vqMax: 0,
        vdMax: 0,
        vqMin: 0,
        vdMin: 0,
        idRef: 0,
        iqRef: 0
    }

    this.rolloffAveragingPeriod = 2;

    this.pwm_enabled = false;

    this.SetDrivemode = function ( val )
    {
        if ( val < this.drivemode.DRIVEMODE_INVALID )
        {
            this.cp.pendingDrivemode = val;
        }
    }

    this.limit = function (val, upper, lower) {
        if ( val > upper ) return upper;
        if ( val < lower ) return lower;
        return val;
    }

    this.SetVDMax = function ( val ) { this.cp.vdMax = val; }
    this.SetVQMax = function ( val ) { this.cp.vqMax = val; }
    this.SetVDMin = function ( val ) { this.cp.vdMin = val; }
    this.SetVQMin = function ( val ) { this.cp.vqMin = val; }

    this.SetKP = function ( val ) { this.piIq.kp = val; this.piId.kp = val; }
    this.SetKI = function ( val ) { this.piIq.ki = val; this.piId.ki = val; }
    this.SetRolloffAveragingPeriod = function ( val ) { this.rolloffAveragingPeriod = val; }

    this.SetIQRef = function ( val ) { this.cp.iqRef = this.limitf(val, this.IQ_LIMIT_A, -this.IQ_LIMIT_A); }
    this.SetIDRef = function ( val ) { this.cp.idRef = this.limitf(val, this.ID_LIMIT_A, -this.ID_LIMIT_A); }

    // =========================================================================================================

    this.foc = new controller.foc();
    this.piId = new controller.pi();
    this.piIq = new controller.pi();

    this.cp = {}
	this.cp.drivemode = this.drivemode.DRIVEMODE_INVALID;
	this.cp.pendingDrivemode = this.drivemode.DRIVEMODE_INVALID;

	this.SetDrivemode( this.drivemode.DRIVEMODE_DISABLED );

	this.SetVDMax( 160.0 );
	this.SetVDMin( -160.0 );
	this.SetVQMax( 160.0 );
	this.SetVQMin( -160.0 );

	this.SetIQRef( 0.0 );
	this.SetIDRef( 0.0 );

	this.piIq.dt = this.DTI;
	this.piId.dt = this.DTI;

	this.pwm_enabled = false;

    this.control_loop = function () {

        this.waitCount = 0;
        this.currentErrorFilter = 0;

        this.waitCount = (this.waitCount + 1)
                    % ( (this.PWM_FREQUENCY_HZ * 2) / this.CONTROL_LOOP_FREQUENCY_HZ );

        if ( this.waitCount == 0 )
        {
            if ( this.cp.pendingDrivemode != this.drivemode.DRIVEMODE_INVALID )
            {
                switch (this.cp.pendingDrivemode)
                {
                    case this.drivemode.DRIVEMODE_FORWARD:
                    case this.drivemode.DRIVEMODE_REVERSE:
                        this.pwm_enabled = true;
                        break;
                    case this.drivemode.DRIVEMODE_DISABLED:
                    default:
                        this.pwm_enabled = false;
                        break;
                }

                this.cp.drivemode = this.cp.pendingDrivemode;
                this.cp.pendingDrivemode = this.drivemode.DRIVEMODE_INVALID;
            }

            // Update the analog measurements.
            UpdateAnalogChannels();

            if ( this.analog_readings.Ia > this.this.SOFTWARE_CURRENT_FAULT_SOFT
              || this.analog_readings.Ia < -this.SOFTWARE_CURRENT_FAULT_SOFT
              || this.analog_readings.Ib > this.SOFTWARE_CURRENT_FAULT_SOFT
              || this.analog_readings.Ib < -this.SOFTWARE_CURRENT_FAULT_SOFT
              || this.analog_readings.Ic > this.SOFTWARE_CURRENT_FAULT_SOFT
              || this.analog_readings.Ic < -this.SOFTWARE_CURRENT_FAULT_SOFT )
            {

                if ( this.analog_readings.Ia > this.SOFTWARE_CURRENT_FAULT_HARD
                      || this.analog_readings.Ia < -this.SOFTWARE_CURRENT_FAULT_HARD
                      || this.analog_readings.Ib > this.SOFTWARE_CURRENT_FAULT_HARD
                      || this.analog_readings.Ib < -this.SOFTWARE_CURRENT_FAULT_HARD
                      || this.analog_readings.Ic > this.SOFTWARE_CURRENT_FAULT_HARD
                      || this.analog_readings.Ic < -this.SOFTWARE_CURRENT_FAULT_HARD )
                {
                    this.errors.push("ERROR_CURRENT_SOFTWARE_FAULT")
                }
                else
                {
                    this.currentErrorFilter++;

                    if( this.currentErrorFilter >= this.SOFTWARE_CURRENT_FAULT_COUNT )
                    {
                        this.errors.push("ERROR_CURRENT_SOFTWARE_FAULT")
                    }
                }

            }
            else if ( this.currentErrorFilter > 0 )
            {
                this.currentErrorFilter--;
            }

            // If there are any errors present, immediately float the gates and put the
            // controller in the disable drivemode.
            if ( this.errors.length > 0 )
            {
                this.pwm_enabled = false;
                this.SetDrivemode(this.drivemode.DRIVEMODE_DISABLED);
                return [-1, -1, -1];
            }

            this.foc.theta = MotorPosition_GetElectricalPosition() - 90;

            if ( this.cp.drivemode == this.drivemode.DRIVEMODE_FORWARD
                || this.cp.drivemode == this.drivemode.DRIVEMODE_REVERSE )
            {
                return this.foc_torque_control();
            }
            else
            {
                var n = this.analog_readings.Va + this.analog_readings.Vb + this.analog_readings.Vc;

                n = n/3;

                var van = this.analog_readings.Va - n;
                var vbn = this.analog_readings.Vb - n

                var valpha = van;
                var vbeta  = ((1/Math.sqrt(3)) * van)
                             + ((2/Math.sqrt(3)) * vbn);
                
                var rad = this.foc.theta * Math.PI / 180;
                var sinTheta = Math.sin(rad);
                var cosTheta = Math.cos(rad);
                
                var vd =  valpha * cosTheta + vbeta * sinTheta;
                var vq = -valpha * sinTheta + vbeta * cosTheta;

                this.foc.Vd = vd;
                this.foc.Vq = vq;

                this.piId.accumulator = vd;
                this.piIq.accumulator = vq;

                return [-1, -1, -1]
            }
        }

    }

    this.foc_torque_control = function ( ) {

        this.brakingEnabled = false;

        this.mechVelocity = MotorPosition_GetMechanicalVelocity();

        this.foc.theta = WrapAngle(foc.theta);

        this.foc.Ia =  this.analog_readings.Ia;
        this.foc.Ib =  this.analog_readings.Ib;
        this.foc.Ic =  this.analog_readings.Ic;
        this.foc.Vbus = this.analog_readings.Vbus;

        if ( this.cp.drivemode == this.drivemode.DRIVEMODE_FORWARD )
        {
            if (this.mechVelocity > 10 )
            {
                this.brakingEnabled = true;
            }
            else if ( this.mechVelocity < 2 )
            {
                this.brakingEnabled = false;
            }

            // If we are braking while going forward, stop braking at 2 RPM
            if (!this.brakingEnabled && this.cp.iqRef < 0 )
            {
                this.cp.iqRef = 0;
            }

            // Filter the input command.
            this.piIq.reference = this.cp.iqRef;
        }
        else if ( this.cp.drivemode == this.drivemode.DRIVEMODE_REVERSE )
        {
            if (this.mechVelocity < -10 )
            {
                this.brakingEnabled = true;
            }
            else if ( this.mechVelocity > -2 )
            {
                this.brakingEnabled = false;
            }

            // If we are braking while going forward, stop braking at 2 RPM
            if (!this.brakingEnabled && this.cp.iqRef < 0 )
            {
                this.cp.iqRef = 0;
            }

            this.piIq.reference = this.cp.iqRef * -1 ;
        }
        else
        {
            // This should never happen.
            this.cp.iqRef = 0;
        }

        this.piId.reference = this.cp.idRef;

        this.foc.sine_cosine();
        this.foc.clarke();
        this.foc.park();

        this.piId.measured = this.foc.Id;
        this.piIq.measured = this.foc.Iq;

        this.piIq.outMax = this.limit( this.foc.Vbus * this.VQ_LIMIT_PERCENT, this.cp.vqMax, -200);
        this.piIq.outMin = this.limit( -this.piIq.outMax, 200, this.cp.vqMin);

        var vqn = this.piIq.control();
        this.foc.Vq = (this.foc.Vq * (this.rolloffAveragingPeriod-1) + vqn) / this.rolloffAveragingPeriod;

        // Find the limit of Vd such that the length of vs does not exceed vbus*VDQ_LIMIT_PERCENT

        var VsLimit = this.foc.Vbus * this.VDQ_LIMIT_PERCENT;

        var vdLimit = Math.sqrt( (VsLimit * VsLimit) - (this.foc.Vq * this.foc.Vq) );

        this.piId.outMax = this.limit( vdLimit, this.cp.vdMax, -200);
        this.piId.outMin = this.limit( -this.piId.outMax, 200, this.cp.vdMin);

        var vdn = this.piId.control();
        this.foc.Vd = (this.foc.Vd * (this.rolloffAveragingPeriod-1) + vdn) / this.rolloffAveragingPeriod;

        this.foc.inverse_park();
        this.foc.inverse_clarke();
        this.foc.svm();

        var oneOverVbus = 1/this.analog_readings.Vbus;

        if (!this.pwm_enabled) return [-1, -1, -1];
        return [ this.foc.V1*oneOverVbus, this.foc.V2*oneOverVbus, this.foc.V3*oneOverVbus ];
    }


    return this;
}




