if (typeof(controller) === "undefined") { window.controller = {}; }

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

    this.sine_cosine = function( ) {
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