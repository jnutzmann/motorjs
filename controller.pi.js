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
