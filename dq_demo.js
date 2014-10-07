
function gen_theta(start, stop, count) {
    var theta = [];

    for (var i=start; i < stop; i += (stop-start)/(count-1)) {
        theta.push(i);
    }

    return theta;
}

function i_phase(Is, theta, offset) {

    function ip(theta) {
        return Math.cos(theta-offset) * Is
    }

    if (typeof(theta) === "object") {
        var result = [];

        for (var i=0; i < theta.length; i++) {
            result.push(ip(theta[i]))
        }

        return result;
    } else {
        return ip(theta);
    }
}

function plot(t, x, label) {
    var data = []

    if (t.length != x.length) {
        console.error("Vectors must be same length.");
    }

    for(var i=0; i < t.length; i++) {
        data.push([t[i], x[i]])
    }

    return { data: data, label:label }
}


function Ia(Is, theta) { return i_phase(Is, theta, 0); }
function Ib(Is, theta) { return i_phase(Is, theta, 2*Math.PI/3); }
function Ic(Is, theta) { return i_phase(Is, theta, 4*Math.PI/3); }


var _Ipeak = 10;
var _Vbus = 100;

var _theta = gen_theta(0, 4*Math.PI, 400);
var _ia = Ia(_Ipeak, _theta);
var _ib = Ib(_Ipeak, _theta);
var _ic = Ic(_Ipeak, _theta);

var foc = new controller.foc();

var _ialpha = [];
var _ibeta = [];
var _id = [];
var _iq = [];

for (var i=0; i < _theta.length; i++) {
    foc.theta = _theta[i];
    foc.Ia = _ia[i];
    foc.Ib = _ib[i];

    foc.sine_cosine_rad();
    foc.clarke();
    foc.park();

    _ialpha.push(foc.Ialpha);
    _ibeta.push(foc.Ibeta);
    _id.push(foc.Id);
    _iq.push(foc.Iq);
}

$(document).ready(function() {
    $.plot($("#plot_theta"), [plot(_theta, _theta, "theta")]);
    $.plot($("#plot_iabc"),
        [
            plot(_theta, _ia, "Ia"),
            plot(_theta, _ib, "Ib"),
            plot(_theta, _ic, "Ic")
        ]
    );
    $.plot($("#plot_ialphabeta"),
        [
            plot(_theta, _ialpha, "Ialpha"),
            plot(_theta, _ibeta, "Ibeta"),
        ]
    );
    $.plot($("#plot_idq"),
        [
            plot(_theta, _id, "Id"),
            plot(_theta, _iq, "Iq"),
        ]
    );
});

