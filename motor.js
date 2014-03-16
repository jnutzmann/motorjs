var dt = 1/60;

var motor = {
  iq: 0,
  loadtype: ''
}
motor.state = new Object();
motor.state.theta = 0;
motor.state.angVel = 0.5;

var load = new Object();
load.car = new Object();
load.car.airDrag = 0.13568;  
load.car.wheelRadius = 0.25;

var defaultParams = {
  polePairs: 6,
  kt: 1,
  kv: 1,
  J: 1,
  drag: 0.1,
  Rs: 0.1
};
motor.params = defaultParams;
function resetMotor() {
  motor.params = defaultParams;
}

var center = $V([0,0]);

var a_ref = $V([1,0]);
var b_ref = a_ref.rotate(Math.PI*2/3,center);
var c_ref = b_ref.rotate(Math.PI*2/3,center);


var a;
var b;
var c;

var d;
var q;

var id;
var iq;

var vd;
var vq;

function angleDifference(x, y) {
  return Math.atan2(Math.sin(x-y), Math.cos(x-y))
}

motor.lookupFlux = function(theta) {
  var i = Math.floor((Math.abs(theta) % (Math.PI*2))/2/Math.PI*this.thetaPrecision);
  return this.fluxALookup[i];
}

motor.regenerate = function() {
  var polePairs = motor.params.polePairs;
  var numWindings = 3 * motor.params.polePairs;
  this.magnets = [];
  
  for (var i = 0; i < (polePairs*2); i++) {
    var magnet = new Object();
    magnet.center = i * Math.PI / polePairs;
    if (i % 2) {
      magnet.B = 1;
    } else {
      magnet.B = -1;
    }
    this.magnets.push(magnet);
  }
  this.fluxALookup = [];
  this.thetaPrecision = 200; // number of lookup points for flux table
  for (var thetaIndex = 0; thetaIndex < this.thetaPrecision; thetaIndex++) {
    var theta = Math.PI * 2 * thetaIndex / this.thetaPrecision;
      var coils = [];
      var coilWidth = Math.PI/numWindings;
      var magnetWidth = motor.params.magnetArc/polePairs;
      for (var i = 0; i < (numWindings); i++) {
        var coil = new Object();
        coil.center = i * Math.PI * 2 / 3 / polePairs;
        coil.flux = 0;
        for (var j = coilWidth/-2; j < coilWidth/2; j += 0.01) {
          var location = j + coil.center;
          for (magnetNum in this.magnets) {
            if (Math.abs(angleDifference(location,this.magnets[magnetNum].center + theta)) < magnetWidth / 2) {
              coil.flux += this.magnets[magnetNum].B;
            }
          }
        }
        coils.push(coil);
      }
    this.fluxALookup.push(coils[0].flux);  
  }
};  

motor.update = function (dt) {
  var numWindings = 3 * motor.params.polePairs;
  var polePairs = motor.params.polePairs;
  
  //this.fluxA = coils[0].flux;
  this.fluxA = this.lookupFlux(motor.state.theta);
  this.emfA = (this.fluxA - this.lastFluxA)*dt;
  this.lastFluxA = this.fluxA;
  
  this.emf = motor.state.angVel * motor.params.kv;
  if (this.drivetype == 'current') {
    this.vq = this.emf + this.iq * this.params.Rs;
  } else {
    this.iq = (this.vq - this.emf) / this.params.Rs;
  }
  
  this.motor_t = this.iq * this.params.kt - this.state.angVel * this.params.drag;
  if (this.loadtype == 'speed') {
    this.loadtorque = -1 * this.motor_t;
  }
    
  if (this.loadtype == 'car') {
    this.loadtorque = 0.5*1.164*motor.state.angVel*motor.state.angVel*load.car.airDrag*load.car.wheelRadius;
    if (this.state.angVel > 0) {
      this.loadtorque *= -1;
    }
  }
  
  var t = this.motor_t + this.loadtorque;
  
  if (this.loadtype == 'speed') {
    this.state.angVel = this.loadVel;
  } else {
    this.state.angVel += (dt * t / motor.params.J);
  }
  
  this.state.theta += this.state.angVel*dt;
  this.state.e_theta = (this.state.theta * this.params.polePairs) % (Math.PI * 2);
}
