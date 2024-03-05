import 'https://cdn.jsdelivr.net/npm/jquery@3.7.1/dist/jquery.min.js';
import 'https://cdn.jsdelivr.net/npm/jquery-ui@1.13.2/dist/jquery-ui.min.js';
import Papa from 'https://cdn.jsdelivr.net/npm/papaparse@5.4.1/+esm'
import {cy,eh,clearCanvas} from './cyto.js';
import {makeAnimation, ANIMATION_ID, PLAYBACK_SPEED} from './animation.js';
import {plotStateData, plotSimData, plotFileData} from './plotting.js';

let JSAT = {
    base: {},
    bodies: {},
    joints: {},
    actuators: {},
    software: {},
    sensors: {},
    gravity: {},
    inports: {}, //only useful for defining models
    outports: {}, //only useful for defining models
    sim: {
        name: "",
        nruns: 0,
        tspan: "(0,10)",
        saveLocation: "",
        dt: ""
    },
};

function jsatConsole(msg) {
    $("#consoleLog").val($("#consoleLog").val() + `\n${msg}`);
}

function jsatError(msg) {
    $("#consoleLog").val($("#consoleLog").val() + msg); //need plugin and to swtich fro mtextarea to div
}

// when enter key is pressed on the element id input "enter", 
// the element id input "click" is clicked
function enterClick(enter, click) {
    enter.on("keyup", function (event) {
        if (event.keyCode === 13) {
            event.preventDefault();
            click.trigger("click");
        }
    })
}


//on blur
$("#simStartTime").on("blur", getSimOptions);
$("#simStopTime").on("blur", getSimOptions);
$("#simName").on("blur", getSimOptions);
$("#simNruns").on("blur", getSimOptions);
$("#simSaveLocation").on("blur", getSimOptions);
$("#simDt").on("blur", getSimOptions);
//on click
$("#simTabButton").on("click", changeTab);
$("#plotTabButton").on("click", changeTab);
$("#animTabButton").on("click", changeTab);
$("#simButton").on("click", sendSimulationData);
$("#addBodyCancelButton").on("click", () => { $("#addBodyDiv").hide() });
$("#addJointCancelButton").on("click", () => { $("#addJointDiv").hide() });
$("#addActuatorCancelButton").on("click", () => { $("#addActuatorDiv").hide() });
$("#addSensorCancelButton").on("click", () => { $("#addSensorDiv").hide() });
$("#addSoftwareCancelButton").on("click", () => { $("#addSoftwareDiv").hide() });
$("#loadSimStates").on("click", getSimStates);
$("#plotState").on("click", plotStateData);
$("#animateBtn").on("click", makeAnimation);
$("#basesButton").on("click", () => { $("#basesLoaderDiv").show() });
$("#basesBackButton").on("click", () => { $("#basesLoaderDiv").hide() });
$("#bodiesButton").on("click", () => { $("#bodiesLoaderDiv").show() });
$("#bodyBackButton").on("click", () => { $("#bodiesLoaderDiv").hide() });
$("#jointsButton").on("click", () => { $("#jointsLoaderDiv").show() });
$("#jointBackButton").on("click", () => { $("#jointsLoaderDiv").hide() });
$("#actuatorsButton").on("click", () => { $("#actuatorLoaderDiv").show() });
$("#actuatorBackButton").on("click", () => { $("#actuatorLoaderDiv").hide() });
$("#softwareButton").on("click", () => { $("#softwareLoaderDiv").show() });
$("#softwareBackButton").on("click", () => { $("#softwareLoaderDiv").hide() });
$("#sensorsButton").on("click", () => { $("#sensorLoaderDiv").show() });
$("#sensorBackButton").on("click", () => { $("#sensorLoaderDiv").hide() });
$("#gravityButton").on("click", () => { $("#gravityLoaderDiv").show() });
$("#gravityBackButton").on("click", () => { $("#gravityLoaderDiv").hide() });
$("#modelsButton").on("click", () => { $("#modelsLoaderDiv").show() });
$("#modelsBackButton").on("click", () => { $("#modelsLoaderDiv").hide() });
$("#portsButton").on("click", () => { $("#portsLoaderDiv").show() });
$("#portsBackButton").on("click", () => { $("#portsLoaderDiv").hide() });
$("#scenariosButton").on("click", () => { $("#scenariosLoaderDiv").show() });
$("#scenariosBackButton").on("click", () => { $("#scenariosLoaderDiv").hide() });
$("#boxButton").on('click', clickAddBoxBody);
$("#cylinderButton").on('click', clickAddCylinderBody);
$('#baseButton').on('click', addBase);
$('#earthButton').on('click', addBaseEarth);
$('#prismaticButton').on('click', clickAddPrismaticJoint);
$('#revoluteButton').on('click', clickAddRevoluteJoint);
$('#floatingButton').on('click', clickAddFloatingJoint);
$('#fixedButton').on('click', clickAddFixedJoint);
$('#thrusterButton').on('click', clickAddActuatorThruster);
$('#reactionWheelButton').on('click', clickAddActuatorRW);
$('#timedCommandButton').on('click', clickAddSoftwareTimedCommand);
$('#customSoftwareButton').on('click', clickAddSoftwareCustom);
$('#constantGravityButton').on('click', clickAddGravityConstant);
$('#drawModeBtn').on('click', toggleDrawMode);
$('#loadFileStates').on('click', () => { $('#loadFileInput').click() });
$('#deleteBtn').on('click', deleteElements);
$('#saveScenarioBtn').on('click', clickSaveScenario);
$('#clearCanvasBtn').on('click', clearCanvas);
$('#simpleAttitudeSensorButton').on('click', clickAddSimpleAttitudeSensor);
$('#simpleAttitudeSensor4Button').on('click', clickAddSimpleAttitudeSensor4);
$('#simpleRateSensorButton').on('click', clickAddSimpleRateSensor);
$('#simpleRateSensor3Button').on('click', clickAddSimpleRateSensor3);
$('#addElementCancelButton').on('click', () => { $('#nameOnlyDiv').hide() })
$("#twoBodyEarthButton").on("click", clickAddGravityTwoBodyEarth);

$("#pbSpeedSlider").on("change", () => { PLAYBACK_SPEED = $("#pbSpeedSlider").val(); })

getSimFileNames();
loadModels();
getScenarios();

cy.on('dbltap', '.body', editBody)
cy.on('dbltap', '.joint', editJoint)
cy.on('dbltap', '.actuator', editActuator)
cy.on('dbltap', '.software', editSoftware)
cy.on('dbltap', '.sensor', editSensor)
cy.on('dbltap', '.gravity', editGravity)

cy.on('dragfree', '.base', function (evt) { JSAT.base.renderedPosition = evt.target.renderedPosition(); });
cy.on('dragfree', '.body', function (evt) { JSAT.bodies[evt.target.data("label")].renderedPosition = evt.target.renderedPosition(); });
cy.on('dragfree', '.joint', function (evt) { JSAT.joints[evt.target.data("label")].renderedPosition = evt.target.renderedPosition(); });
cy.on('dragfree', '.actuator', function (evt) { JSAT.actuators[evt.target.data("label")].renderedPosition = evt.target.renderedPosition(); });
cy.on('dragfree', '.software', function (evt) { JSAT.software[evt.target.data("label")].renderedPosition = evt.target.renderedPosition(); });
cy.on('dragfree', '.sensor', function (evt) { JSAT.sensors[evt.target.data("label")].renderedPosition = evt.target.renderedPosition(); });
cy.on('dragfree', '.gravity', function (evt) { JSAT.gravity[evt.target.data("label")].renderedPosition = evt.target.renderedPosition(); });

cy.on('ehcomplete', (evt, src, tar, edge) => {
    //error handling
    if (src.data().type === "out") {
        jsatConsole("outport cannot be arrow base, must be arrow head")
        edge.remove();
        return;
    }

    if (tar.data().type === "in") {
        jsatConsole("inport cannot be arrow head, must be arrow base")
        edge.remove();
        return;
    }

    //set if predecessor
    if (src.classes().includes("body")) {
        if (tar.classes().includes("joint")) {
            const source_id = src.data().label;
            const target_id = tar.data().label;
            JSAT.joints[target_id]["predecessor"] = source_id;
        } else if (tar.classes().includes("actuator")) {
            jsatConsole("connect actuators to bodies, not bodys to actuators")
            edge.remove();
            return;
        } else if (tar.classes().includes("gravity")) {
            jsatConsole("connect environments to bodies, not bodies to environments")
            edge.remove();
            return;
        } else if (tar.classes().includes("sensor")) {
            jsatConsole("connect sensors to bodies, not bodies to sensors")
        }

    }

    if (src.classes().includes("actuator")) {
        if (tar.classes().includes("body")) {
            const source_id = src.data().label;
            const target_id = tar.data().label;
            JSAT.bodies[target_id]["actuators"].push(source_id);
        } else {
            jsatConsole("actuators can only connect to bodies")
            edge.remove();
            return;
        }
    }

    if (src.classes().includes("gravity")) {
        const source_id = src.data().label;
        const target_id = tar.data().label;
        if (tar.classes().includes("body")) {
            JSAT.bodies[target_id]["gravity"].push(source_id);
        } else if (tar.classes().includes("base")) {
            JSAT.base.gravity.push(source_id)
        } else {
            jsatConsole("environments can only connect to bodies")
            edge.remove();
            return;
        }
    }

    if (src.classes().includes("software")) {
        if (tar.classes().includes("actuator")) {
            const source_id = src.data().label;
            const target_id = tar.data().label;
            JSAT.actuators[target_id]["command"] = source_id;
            JSAT.software[source_id].actuators.push(target_id);
        } else {
            jsatConsole("software only connects to actuators and other software")
        }
    }

    if (src.classes().includes("sensor")) {
        if (tar.classes().includes("body")) {
            const source_id = src.data().label;
            const target_id = tar.data().label;
            JSAT.bodies[target_id].sensors.push(source_id);
        } else if (tar.classes().includes("software")) {
            const source_id = src.data().label;
            const target_id = tar.data().label;
            JSAT.software[target_id].sensors.push(source_id);
        } else {
            jsatConsole("sensors only connect to bodies and software")
        }
    }

    //set if predecessor
    if (src.classes().includes("port")) {
        const source_id = src.data().label;
        const target_id = tar.data().label;
        JSAT.inports[source_id].successor = target_id;
        if (tar.classes().includes("joint")) {
            JSAT.joints[target_id]["predecessor"] = source_id;
        }
        if (tar.classes().includes("port")) {
            JSAT.outports[target_id]["predecessor"] = source_id;
        }
    }

    //set if successor
    if (src.classes().includes("joint")) {
        if (tar.classes().includes("body")) {
            const source_id = src.data().label;
            const target_id = tar.data().label;
            JSAT.joints[source_id]["successor"] = target_id;
        }
    }

    //set if base
    if (src.classes().includes("base")) {
        if (tar.classes().includes("joint")) {
            const source_id = src.data().label;
            const target_id = tar.data().label;
            JSAT.joints[target_id]["predecessor"] = source_id;
        }
    }
    console.log(JSAT)

});


function toggleDrawMode() {
    if (eh.drawMode) {
        eh.disableDrawMode();
        $('#drawModeBtn').removeClass("active-border")
        $('#drawModeBtn').addClass("not-active-border")
    } else {
        eh.enableDrawMode();
        $('#drawModeBtn').removeClass("not-active-border")
        $('#drawModeBtn').addClass("active-border")
    }
}

function addBase() {
    let height = cy.height()
    let width = cy.width()
    if (cy.$('.base').length > 0) {
        jsatConsole('cant have more than 1 base!')
    } else {
        cy.add({
            group: 'nodes',
            data: {
                id: 'base',
                label: 'base',
                jsatTarget: JSAT.base
            },
            classes: 'base',
            renderedPosition: {
                x: width / 2,
                y: height / 2,
            },
        });
    }
    JSAT.base = {
        type: "default",
        gravity: [],
        renderedPosition: cy.$('#base').renderedPosition()
    }
    console.log(JSAT)
}


function addBaseEarth() {
    let height = cy.height()
    let width = cy.width()
    if (cy.$('.base').length > 0) {
        jsatConsole('cant have more than 1 base!')
    } else {
        cy.add({
            group: 'nodes',
            data: {
                id: 'earth',
                label: 'earth'
            },
            classes: 'base',
            renderedPosition: {
                x: width / 2,
                y: height / 2,
            },
        });
    }
    JSAT.base = {
        type: "earth",
        gravity: [],
        renderedPosition: cy.$('#earth').renderedPosition()
    }
}

function addBodyBoxInputs() {
    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font " for="newBodyXLength">x length:</label><br></td> \
            <td><input id="newBodyXLength" class="form-input" type="text" name="newXBodyLength" placeholder="1"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font " for="newBodyYLength">y length:</label><br></td> \
            <td><input id="newBodyYLength" class="form-input" type="text" name="newYBodyLength" placeholder="1"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font " for="newBodyZLength">z length:</label><br></td> \
            <td><input id="newBodyZLength" class="form-input" type="text" name="newZBodyLength" placeholder="1"><br></td>\
        </tr>');
}

function clickAddBoxBody() {
    //remove any inputs already there
    $('.geometry-input').remove();

    // add box geometry specifc inputs
    addBodyBoxInputs()

    // bind box to save event, mark as new 
    $("#addBodySaveButton").off()
    $("#addBodySaveButton").on("click", { new: true, geometry: 'box', name: '' }, saveBody)
    //show the details div
    $('#addBodyDiv').show();
}

function addBodyCylinderInputs() {
    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font " for="newBodyRadiusTop">radius top:</label><br></td> \
            <td><input id="newBodyRadiusTop" class="form-input" type="text" name="newBodyRadiusTop" placeholder="1"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font " for="newBodyRadiusBottom">radius bottom:</label><br></td> \
            <td><input id="newBodyRadiusBottom" class="form-input" type="text" name="newBodyRadiusBottom" placeholder="1"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font " for="newBodyHeight">height:</label><br></td> \
            <td><input id="newBodyHeight" class="form-input" type="text" name="newBodyHeight" placeholder="1"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font">radial segments:</label><br></td> \
            <td><input id="newBodyRadialSegments" class="form-input" type="text" placeholder="32"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font">height segments:</label><br></td> \
            <td><input id="newBodyHeightSegments" class="form-input" type="text" placeholder="1"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font">open ended:</label><br></td> \
            <td><input id="newBodyOpenEnded" class="form-input" type="text" placeholder="capped"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font">theta start:</label><br></td> \
            <td><input id="newBodyThetaStart" class="form-input" type="text" placeholder="0"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font">theta length:</label><br></td> \
            <td><input id="newBodyThetaLength" class="form-input" type="text" placeholder="2*pi"><br></td>\
        </tr>');
}

function clickAddCylinderBody() {

    //remove any inputs already there
    $('.geometry-input').remove();

    // add cylinder specific inputs to the add body table 
    addBodyCylinderInputs();

    // bind box to save event, mark as new 
    $("#addBodySaveButton").off()
    $("#addBodySaveButton").on("click", { new: true, geometry: 'cylinder', name: '' }, saveBody)
    //show the details div
    $('#addBodyDiv').show();
}

function saveBody(event) {
    let body = {
        name: $("#newBodyName").val(),
        mass: { nominal: $("#newBodyMass").val(), dispersed: $("#newBodyMassDist").val() },
        cmx: { nominal: $("#newBodyCmX").val(), dispersed: $("#newBodyCmXDist").val() },
        cmy: { nominal: $("#newBodyCmY").val(), dispersed: $("#newBodyCmYDist").val() },
        cmz: { nominal: $("#newBodyCmZ").val(), dispersed: $("#newBodyCmZDist").val() },
        ixx: { nominal: $("#newBodyIxx").val(), dispersed: $("#newBodyIxxDist").val() },
        iyy: { nominal: $("#newBodyIyy").val(), dispersed: $("#newBodyIyyDist").val() },
        izz: { nominal: $("#newBodyIzz").val(), dispersed: $("#newBodyIzzDist").val() },
        ixy: { nominal: $("#newBodyIxy").val(), dispersed: $("#newBodyIxyDist").val() },
        ixz: { nominal: $("#newBodyIxz").val(), dispersed: $("#newBodyIxzDist").val() },
        iyz: { nominal: $("#newBodyIyz").val(), dispersed: $("#newBodyIyzDist").val() },
        geometry: event.data.geometry,
        material: $("#newBodyMaterial").val(),
        color: $("#newBodyColor").val(),
        actuators: [],
        sensors: [],
        gravity: [],
        environments: [],
        renderedPosition: {}
    };

    //defaults
    if (body.mass.nominal === "") { body.mass.nominal = "1" }
    if (body.cmx.nominal === "") { body.cmx.nominal = "0" }
    if (body.cmy.nominal === "") { body.cmy.nominal = "0" }
    if (body.cmz.nominal === "") { body.cmz.nominal = "0" }
    if (body.ixx.nominal === "") { body.ixx.nominal = "1" }
    if (body.iyy.nominal === "") { body.iyy.nominal = "1" }
    if (body.izz.nominal === "") { body.izz.nominal = "1" }
    if (body.ixy.nominal === "") { body.ixy.nominal = "0" }
    if (body.ixz.nominal === "") { body.ixz.nominal = "0" }
    if (body.iyz.nominal === "") { body.iyz.nominal = "0" }
    if (body.material === "") { body.material = "basic" }
    if (body.color === "") { body.color = "aquamarine" }

    if (event.data.geometry === 'box') {
        body['xlength'] = $("#newBodyXLength").val();
        body['ylength'] = $("#newBodyYLength").val();
        body['zlength'] = $("#newBodyZLength").val();

        //defaults
        if (body.xlength === "") { body.xlength = 1 }
        if (body.ylength === "") { body.ylength = 1 }
        if (body.zlength === "") { body.zlength = 1 }
    }

    if (event.data.geometry === 'cylinder') {
        body['radiusTop'] = $('#newBodyRadiusTop').val();
        body['radiusBottom'] = $('#newBodyRadiusBottom').val();
        body['height'] = $('#newBodyHeight').val();
        body['radialSegments'] = $('#newBodyRadialSegments').val();
        body['heightSegments'] = $('#newBodyHeightSegments').val();
        body['openEnded'] = $('#newBodyOpenEnded').val();
        body['thetaStart'] = $('#newBodyThetaStart').val();
        body['thetaLength'] = $('#newBodyThetaLength').val();

        //defaults
        if (body.radiusTop === "") { body.radiusTop = 1 }
        if (body.radiusBottom === "") { body.radiusBottom = 1 }
        if (body.height === "") { body.height = 1 }
        if (body.radialSegments === "") { body.radialSegments = 32 }
        if (body.heightSegments === "") { body.heightSegments = 1 }
        if (body.openEnded === "") { body.openEnded = false }
        if (body.thetaStart === "") { body.thetaStart = 0 }
        if (body.thetaLength === "") { body.thetaLength = 2 * Math.PI }
    }

    // create button if this is a new body        
    const name = $("#newBodyName").val();
    const height = cy.height();
    const width = cy.width();
    if (event.data.new) {
        cy.add({
            group: 'nodes',
            data: {
                id: `body${name}`,
                label: name,
            },
            classes: 'body',
            renderedPosition: {
                x: width / 2,
                y: height / 2,
            },
        });
    } else {
        //update node data
        cy.$(`#body${event.data.name}`).data('id', `body${name}`)
        cy.$(`#body${event.data.name}`).data('label', name)

        body.actuators = JSAT.bodies[body.name].actuators
        body.sensors = JSAT.bodies[body.name].sensors
        body.gravity = JSAT.bodies[body.name].gravity
        body.environments = JSAT.bodies[body.name].environments
        delete JSAT.bodies[event.data.name]
    }
    body.renderedPosition = cy.$(`#body${name}`).renderedPosition();

    JSAT.bodies[body.name] = body;
    console.log(JSAT)
    $('#addBodyDiv').hide();
}

function editBody() {
    //remove any previously stored geometry specific inputs
    $('.geometry-input').remove();

    const name = this.data().label;
    const body = JSAT.bodies[name];
    $("#newBodyName").val(body.name);
    $("#newBodyMass").val(body.mass.nominal);
    $("#newBodyMassDist").val(body.mass.dispersed);
    $("#newBodyCmX").val(body.cmx.nominal);
    $("#newBodyCmXDist").val(body.cmx.dispersed);
    $("#newBodyCmY").val(body.cmy.nominal);
    $("#newBodyCmYDist").val(body.cmy.dispersed);
    $("#newBodyCmZ").val(body.cmz.nominal);
    $("#newBodyCmZDist").val(body.cmz.dispersed);
    $("#newBodyIxx").val(body.ixx.nominal);
    $("#newBodyIxxDist").val(body.ixx.dispersed);
    $("#newBodyIyy").val(body.iyy.nominal);
    $("#newBodyIyyDist").val(body.iyy.dispersed);
    $("#newBodyIzz").val(body.izz.nominal);
    $("#newBodyIzzDist").val(body.izz.dispersed);
    $("#newBodyIxy").val(body.ixy.nominal);
    $("#newBodyIxyDist").val(body.ixy.dispersed);
    $("#newBodyIxz").val(body.ixz.nominal);
    $("#newBodyIxzDist").val(body.ixz.dispersed);
    $("#newBodyIyz").val(body.iyz.nominal);
    $("#newBodyIyzDist").val(body.iyz.dispersed);
    $("#newBodyMaterial").val(body.material);
    $("#newBodyColor").val(body.color);

    if (body.geometry === 'box') {
        addBodyBoxInputs();
        $("#newBodyXLength").val(body.xlength);
        $("#newBodyYLength").val(body.ylength);
        $("#newBodyZLength").val(body.zlength);
    }

    if (body.geometry === 'cylinder') {
        addBodyCylinderInputs();
        $('#newBodyRadiusTop').val(body.radiusTop);
        $('#newBodyRadiusBottom').val(body.radiusBottom);
        $('#newBodyHeight').val(body.height);
        $('#newBodyRadialSegments').val(body.radialSegments);
        $('#newBodyHeightSegments').val(body.heightSegments);
        $('#newBodyOpenEnded').val(body.openEnded);
        $('#newBodyThetaStart').val(body.thetaStart);
        $('#newBodyThetaLength').val(body.thetaLength);
    }

    $("#addBodySaveButton").off();
    $("#addBodySaveButton").on("click", { new: false, geometry: body.geometry, name: name }, saveBody)
    $("#addBodyDiv").show();
}

function addJointRevoluteInputs() {
    $('#jointTable tbody').append("<tr class = 'joint-input'> \
            <td><label class='form-font'>&theta;:</label><br></td> \
            <td><input id='newJointTheta' class='form-input' type='text' placeholder='0'><br></td>\
        </tr>");

    $('#jointTable tbody').append("<tr class = 'joint-input'> \
            <td><label class='form-font'>&omega;:</label><br></td> \
            <td><input id='newJointOmega' class='form-input' type='text' placeholder='0'><br></td>\
        </tr>");

    $('#jointTable tbody').append("<tr class = 'joint-input'> \
        <td><label class='form-font'>f:</label><br></td> \
        <td><input id='newJointForce' class='form-input' type='text' placeholder='0'><br></td>\
    </tr>");

    $('#jointTable tbody').append("<tr class = 'joint-input'> \
        <td><label class='form-font'>&kappa;:</label><br></td> \
        <td><input id='newJointKappa' class='form-input' type='text' placeholder='0'><br></td>\
    </tr>");

    $('#jointTable tbody').append("<tr class = 'joint-input'> \
        <td><label class='form-font'>&zeta;:</label><br></td> \
        <td><input id='newJointZeta' class='form-input' type='text' placeholder='0'><br></td>\
    </tr>");

    $('#jointTable tbody').append("<tr class = 'joint-input'> \
        <td><label class='form-font'>pos lower limit:</label><br></td> \
        <td><input id='newJointPosLowLim' class='form-input' type='text' placeholder='-Inf'><br></td>\
    </tr>");

    $('#jointTable tbody').append("<tr class = 'joint-input'> \
        <td><label class='form-font'>pos upper limit:</label><br></td> \
        <td><input id='newJointPosUpLim' class='form-input' type='text' placeholder='Inf'><br></td>\
    </tr>");

}

function addJointFloatingInputs() {
    $('#jointTable tbody').append("<tr class = 'joint-input'> \
            <td><label class='form-font'>quaternion:</label><br></td> \
            <td><input id='newJointQuat' class='form-input' type='text' placeholder='[0,0,0,1]'><br></td>\
        </tr>");

    $('#jointTable tbody').append("<tr class = 'joint-input'> \
            <td><label class='form-font'>&omega;:</label><br></td> \
            <td><input id='newJointOmega' class='form-input' type='text' placeholder='zeros(3)'><br></td>\
        </tr>");

    $('#jointTable tbody').append("<tr class = 'joint-input'> \
            <td><label class='form-font'>position:</label><br></td> \
            <td><input id='newJointPosition' class='form-input' type='text' placeholder='zeros(3)'><br></td>\
        </tr>");

    $('#jointTable tbody').append("<tr class = 'joint-input'> \
            <td><label class='form-font'>velocity:</label><br></td> \
            <td><input id='newJointVelocity' class='form-input' type='text' placeholder='zeros(3)'><br></td>\
        </tr>");

}

function addJointFixedInputs() {
    $('#jointTable tbody').append("<tr class = 'joint-input'> \
            <td><label class='form-font'>quaternion:</label><br></td> \
            <td><input id='newJointQuat' class='form-input' type='text' placeholder='[0,0,0,1]'><br></td>\
        </tr>");

    $('#jointTable tbody').append("<tr class = 'joint-input'> \
            <td><label class='form-font'>position:</label><br></td> \
            <td><input id='newJointPosition' class='form-input' type='text' placeholder='zeros(3)'><br></td>\
        </tr>");

}

function addJointPrismaticInputs() {
    $('#jointTable tbody').append("<tr class = 'joint-input'> \
            <td><label class='form-font'>position:</label><br></td> \
            <td><input id='newJointPosition' class='form-input' type='text' placeholder='0'><br></td>\
        </tr>");

    $('#jointTable tbody').append("<tr class = 'joint-input'> \
            <td><label class='form-font'>velocity:</label><br></td> \
            <td><input id='newJointVelocity' class='form-input' type='text' placeholder='0'><br></td>\
        </tr>");

}
function clickAddPrismaticJoint() {
    //remove all old inputs
    $('.joint-input').remove();
    //add revolute specific inputs
    addJointPrismaticInputs();
    // bind revolute to save event, mark as new 
    $("#addJointSaveButton").off()
    $("#addJointSaveButton").on("click", { new: true, type: 'prismatic', name: '' }, saveJoint)
    //show the details div
    $('#addJointDiv').show();
}


function clickAddRevoluteJoint() {
    //remove all old inputs
    $('.joint-input').remove();
    //add revolute specific inputs
    addJointRevoluteInputs();
    // bind revolute to save event, mark as new 
    $("#addJointSaveButton").off()
    $("#addJointSaveButton").on("click", { new: true, type: 'revolute', name: '' }, saveJoint)
    //show the details div
    $('#addJointDiv').show();
}

function clickAddFloatingJoint() {
    //remove all old inputs
    $('.joint-input').remove();
    //add revolute specific inputs
    addJointFloatingInputs();
    // bind revolute to save event, mark as new 
    $("#addJointSaveButton").off()
    $("#addJointSaveButton").on("click", { new: true, type: 'floating', name: '' }, saveJoint)
    //show the details div
    $('#addJointDiv').show();
}

function clickAddFixedJoint() {
    //remove all old inputs
    $('.joint-input').remove();
    //add fixed joint specific inputs
    addJointFixedInputs()
    // bind revolute to save event, mark as new 
    $("#addJointSaveButton").off()
    $("#addJointSaveButton").on("click", { new: true, type: 'fixed', name: '' }, saveJoint)
    //show the details div
    $('#addJointDiv').show();
}

function saveJoint(event) {
    const name = $("#newJointName").val();

    let joint = {
        name: name,
        type: event.data.type,
        FpRho: $("#newJointFpRho").val(),
        FpPhi: $("#newJointFpPhi").val(),
        FsRho: $("#newJointFsRho").val(),
        FsPhi: $("#newJointFsPhi").val(),
        renderedPosition: {}
    };

    if (event.data.new) {
        joint.predecessor = "undef"; //defined after connection    
        joint.successor = "undef";  //defined after connection
    } else {
        joint.predecessor = event.data.predecessor;
        joint.successor = event.data.successor;
    }

    //defaults
    if (joint.FpRho === "") { joint.FpRho = "zeros(3)" }
    if (joint.FsRho === "") { joint.FsRho = "zeros(3)" }
    if (joint.FpPhi === "") { joint.FpPhi = "I(3)" }
    if (joint.FsPhi === "") { joint.FsPhi = "I(3)" }


    if (event.data.type === 'revolute') {
        joint['theta'] = $("#newJointTheta").val();
        joint['omega'] = $("#newJointOmega").val();
        joint['force'] = $("#newJointForce").val();
        joint['kappa'] = $("#newJointKappa").val();
        joint['zeta'] = $("#newJointZeta").val();
        joint['poslowlim'] = $("#newJointPosLowLim").val();
        joint['posuplim'] = $("#newJointPosUpLim").val();

        //defaults
        if (joint.theta === "") { joint.theta = "0" }
        if (joint.omega === "") { joint.omega = "0" }
        if (joint.force === "") { joint.force = "0" }
        if (joint.kappa === "") { joint.kappa = "0" }
        if (joint.zeta === "") { joint.zeta = "0" }
        if (joint.poslowlim === "") { joint.poslowlim = "-Inf" }
        if (joint.posuplim === "") { joint.posuplim = "Inf" }
    }

    if (event.data.type === 'prismatic') {
        joint['position'] = $("#newJointPosition").val();
        joint['velocity'] = $("#newJointVelocity").val();

        //defaults
        if (joint.position === "") { joint.position = "0" }
        if (joint.velocity === "") { joint.velocity = "0" }
    }

    if (event.data.type === 'floating') {
        joint['q'] = $("#newJointQuat").val();
        joint['omega'] = $("#newJointOmega").val();
        joint['position'] = $("#newJointPosition").val();
        joint['velocity'] = $("#newJointVelocity").val();

        //defaults
        if (joint.q === "") { joint.q = "[0,0,0,1]" }
        if (joint.omega === "") { joint.omega = "zeros(3)" }
        if (joint.position === "") { joint.position = "zeros(3)" }
        if (joint.velocity === "") { joint.velocity = "zeros(3)" }
    }

    if (event.data.type === 'fixed') {
        joint['q'] = $("#newJointQuat").val();
        joint['position'] = $("#newJointPosition").val();

        //defaults
        if (joint.q === "") { joint.q = "[0,0,0,1]" }
        if (joint.position === "") { joint.position = "zeros(3)" }
    }

    if (event.data.new) {
        const height = cy.height();
        const width = cy.width();
        cy.add({
            group: 'nodes',
            data: {
                id: `joint${name}`,
                label: name,
            },
            classes: 'joint',
            renderedPosition: {
                x: width / 2,
                y: height / 2,
            },
        });
    } else {
        cy.$(`#joint${event.data.name}`).data('id', `joint${name}`)
        cy.$(`#joint${event.data.name}`).data('label', name)

        delete JSAT.joints[event.data.name]
    }
    joint.renderedPosition = cy.$(`joint${name}`).renderedPosition();

    JSAT.joints[name] = joint;
    console.log(JSAT);
    $('#addJointDiv').hide();
}

function editJoint() {
    //remove all previously stored joint specific inputs
    $('.joint-input').remove();

    const name = this.data().label;
    const joint = JSAT.joints[name];

    $("#newJointName").val(joint.name);
    $("#newJointFpPhi").val(joint.FpPhi);
    $("#newJointFpRho").val(joint.FpRho);
    $("#newJointFsPhi").val(joint.FsPhi);
    $("#newJointFsRho").val(joint.FsRho);

    if (joint.type === 'revolute') {
        addJointRevoluteInputs();
        $("#newJointTheta").val(joint.theta);
        $("#newJointOmega").val(joint.omega);
        $("#newJointForce").val(joint.force);
        $("#newJointKappa").val(joint.kappa);
        $("#newJointZeta").val(joint.zeta);
        $("#newJointPosLowLim").val(joint.poslowlim);
        $("#newJointPosUpLim").val(joint.posuplim);
    }

    if (joint.type === 'prismatic') {
        addJointPrismaticInputs();
        $("#newJointPosition").val(joint.position);
        $("#newJointVelocity").val(joint.velocity);
    }

    if (joint.type === 'floating') {
        addJointFloatingInputs();
        $("#newJointQuat").val(joint.q);
        $("#newJointOmega").val(joint.omega);
        $("#newJointPosition").val(joint.position);
        $("#newJointVelocity").val(joint.velocity);
    }

    if (joint.type === 'fixed') {
        addJointFixedInputs();
        $("#newJointQuat").val(joint.q);
        $("#newJointPosition").val(joint.position);
    }

    $("#addJointSaveButton").off();
    $("#addJointSaveButton").on("click", { new: false, type: joint.type, name: name, predecessor: joint.predecessor, successor: joint.successor }, saveJoint)
    $("#addJointDiv").show();

};

function clickAddSimpleAttitudeSensor() {
    //remove all old inputs
    $('.sensor-input').remove();
    // bind sensor to save event, mark as new 
    $("#addSensorSaveButton").off()
    $("#addSensorSaveButton").on("click", { new: true, type: 'simpleAttitudeSensor', name: '' }, saveSensor)
    //show the details div
    $('#addSensorDiv').show();
}


function clickAddSimpleAttitudeSensor4() {
    //remove all old inputs
    $('.sensor-input').remove();
    // bind sensor to save event, mark as new 
    $("#addSensorSaveButton").off()
    $("#addSensorSaveButton").on("click", { new: true, type: 'simpleAttitudeSensor4', name: '' }, saveSensor)
    //show the details div
    $('#addSensorDiv').show();
}

function clickAddSimpleRateSensor() {
    //remove all old inputs
    $('.sensor-input').remove();
    // bind sensor to save event, mark as new 
    $("#addSensorSaveButton").off()
    $("#addSensorSaveButton").on("click", { new: true, type: 'simpleRateSensor', name: '' }, saveSensor)
    //show the details div
    $('#addSensorDiv').show();
}

function clickAddSimpleRateSensor3() {
    //remove all old inputs
    $('.sensor-input').remove();
    // bind sensor to save event, mark as new 
    $("#addSensorSaveButton").off()
    $("#addSensorSaveButton").on("click", { new: true, type: 'simpleRateSensor3', name: '' }, saveSensor)
    //show the details div
    $('#addSensorDiv').show();
}


function saveSensor(event) {
    const name = $("#newSensorName").val();

    let sensor = {
        name: name,
        type: event.data.type,
        rotation: $("#newSensorRotation").val(),
        translation: $("#newSensorTranslation").val(),
        body: "undef",
        renderedPosition: {}
    };

    //defaults
    if (sensor.rotation === "") { sensor.rotation = "I(3)" }
    if (sensor.translation === "") { sensor.translation = "zeros(3)" }

    if (event.data.new) {
        const height = cy.height();
        const width = cy.width();

        cy.add({
            group: 'nodes',
            data: {
                id: `sensor${name}`,
                label: name,
            },
            classes: 'sensor',
            renderedPosition: {
                x: width / 2,
                y: height / 2,
            },
        });
    } else {
        cy.$(`#sensor${event.data.name}`).data('id', `sensor${name}`)
        cy.$(`#sensor${event.data.name}`).data('label', name)

        sensor.body = JSAT.sensors[event.data.name].body
        delete JSAT.sensors[event.data.name]
    }
    sensor.renderedPosition = cy.$(`#sensor${name}`).renderedPosition();

    JSAT.sensors[name] = sensor;
    console.log(JSAT);
    $('#addSensorDiv').hide();
}

function editSensor() {
    //remove all previously stored joint specific inputs
    $('.sensor-input').remove();

    const name = this.data().label;
    const sensor = JSAT.sensors[name];

    $("#newSensorName").val(sensor.name);
    $("#newSensorRotation").val(sensor.rotation);
    $("#newSensorTranslation").val(sensor.translation);

    $("#addSensorSaveButton").off();
    $("#addSensorSaveButton").on("click", { new: false, type: sensor.type, name: name }, saveSensor)
    $("#addSensorDiv").show();
};

function addActuatorThrusterInputs() {
    $('#actuatorTable tbody').append("<tr class = 'actuator-input'> \
            <td><label class='form-font'>thrust magnitude:</label><br></td> \
            <td><input id='newActuatorThrust' class='form-input' type='text' placeholder='1'><br></td>\
        </tr>");
}

function clickAddActuatorThruster() {
    //remove all old inputs
    $('.actuator-input').remove();
    //add actuator specific inputs
    addActuatorThrusterInputs();
    // bind actuator to save event, mark as new 
    $("#addActuatorSaveButton").off()
    $("#addActuatorSaveButton").on("click", { new: true, type: 'thruster', name: '' }, saveActuator)
    //show the details div
    $('#addActuatorDiv').show();
}

function addActuatorRWInputs() {
    $('#actuatorTable tbody').append("<tr class = 'actuator-input'> \
            <td><label class='form-font'>inertia:</label><br></td> \
            <td><input id='newActuatorInertia' class='form-input' type='text' placeholder='0.25'><br></td>\
        </tr>");

    $('#actuatorTable tbody').append("<tr class = 'actuator-input'> \
            <td><label class='form-font'>motor constant:</label><br></td> \
            <td><input id='newActuatorKt' class='form-input' type='text' placeholder='0.075'><br></td>\
        </tr>");

    $('#actuatorTable tbody').append("<tr class = 'actuator-input'> \
        <td><label class='form-font'>initial momentum:</label><br></td> \
        <td><input id='newActuatorH' class='form-input' type='text' placeholder='0'><br></td>\
    </tr>");
}

function clickAddActuatorRW() {
    //remove all old inputs
    $('.actuator-input').remove();
    //add actuator specific inputs
    addActuatorRWInputs();
    // bind actuator to save event, mark as new 
    $("#addActuatorSaveButton").off()
    $("#addActuatorSaveButton").on("click", { new: true, type: 'reactionWheel', name: '' }, saveActuator)
    //show the details div
    $('#addActuatorDiv').show();
}


function saveActuator(event) {
    const name = $("#newActuatorName").val();

    let actuator = {
        name: name,
        type: event.data.type,
        rotation: $("#newActuatorRotation").val(),
        translation: $("#newActuatorTranslation").val(),
        command: "undef",
        renderedPosition: {}
    };

    //defaults
    if (actuator.rotation === "") { actuator.rotation = "I(3)" }
    if (actuator.translation === "") { actuator.translation = "zeros(3)" }

    if (event.data.type === 'thruster') {
        actuator['thrust'] = $("#newActuatorThrust").val();

        //defaults
        if (actuator.thrust === "") { actuator.thrust = "1" }
    }

    if (event.data.type === 'reactionWheel') {
        actuator['inertia'] = $("#newActuatorInertia").val();
        actuator['kt'] = $("#newActuatorKt").val();
        actuator['H'] = $("#newActuatorH").val();

        //defaults
        if (actuator.inertia === "") { actuator.inertia = "0.25" }
        if (actuator.kt === "") { actuator.kt = "0.075" }
        if (actuator.H === "") { actuator.H = "0" }
    }

    if (event.data.new) {
        const height = cy.height();
        const width = cy.width();
        cy.add({
            group: 'nodes',
            data: {
                id: `actuator${name}`,
                label: name,
            },
            classes: 'actuator',
            renderedPosition: {
                x: width / 2,
                y: height / 2,
            },
        });
    } else {
        cy.$(`#actuator${event.data.name}`).data('id', `actuator${name}`)
        cy.$(`#actuator${event.data.name}`).data('label', name)

        actuator.command = JSAT.actuators[event.data.name].command
        delete JSAT.actuators[event.data.name]
    }
    actuator.renderedPosition = cy.$(`#actuator${name}`).renderedPosition();

    JSAT.actuators[name] = actuator;
    console.log(JSAT);
    $('#addActuatorDiv').hide();
}

function editActuator() {
    //remove all previously stored joint specific inputs
    $('.actuator-input').remove();

    const name = this.data().label;
    const actuator = JSAT.actuators[name];

    $("#newActuatorName").val(actuator.name);
    $("#newActuatorRotation").val(actuator.rotation);
    $("#newActuatorTranslation").val(actuator.translation);

    if (actuator.type === 'thruster') {
        addActuatorThrusterInputs();
        $("#newActuatorThrust").val(actuator.thrust);
    }

    if (actuator.type === 'reactionWheel') {
        addActuatorRWInputs();
        $("#newActuatorInertia").val(actuator.inertia);
        $("#newActuatorKt").val(actuator.kt);
        $("#newActuatorH").val(actuator.H);
    }

    $("#addActuatorSaveButton").off();
    $("#addActuatorSaveButton").on("click", { new: false, type: actuator.type, name: name }, saveActuator)
    $("#addActuatorDiv").show();

};

function addSoftwareTimedCommandInputs() {
    $('#softwareTable tbody').append("<tr class = 'software-input'> \
            <td><label class='form-font'>step times:</label><br></td> \
            <td><input id='newSoftwareTSteps' class='form-input' type='text' placeholder='[]'><br></td>\
        </tr>");

    $('#softwareTable tbody').append("<tr class = 'software-input'> \
            <td><label class='form-font'>step values:</label><br></td> \
            <td><input id='newSoftwareValues' class='form-input' type='text' placeholder='[]'><br></td>\
        </tr>");
}


function clickAddSoftwareTimedCommand() {
    //remove all old inputs
    $('.software-input').remove();
    //add software specific inputs
    addSoftwareTimedCommandInputs();
    // bind actuator to save event, mark as new 
    $("#addSoftwareSaveButton").off()
    $("#addSoftwareSaveButton").on("click", { new: true, type: 'timedCommand', name: '' }, saveSoftware)
    //show the details div
    $('#addSoftwareDiv').show();
}

function addSoftwareCustomInputs() {
    $('#softwareTable tbody').append("<tr class = 'software-input'> \
            <td><label class='form-font'>module:</label><br></td> \
            <td><select id='newSoftwareModule' class='select' placeholder='select'><br></td>\
        </tr>");

    getCustomSoftware();
}

function clickAddSoftwareCustom() {

    //remove all old inputs
    $('.software-input').remove();
    //add software specific inputs
    addSoftwareCustomInputs();
    // bind actuator to save event, mark as new 
    $("#addSoftwareSaveButton").off()
    $("#addSoftwareSaveButton").on("click", { new: true, type: 'custom', name: '' }, saveSoftware)
    //show the details div
    $('#addSoftwareDiv').show();
}

function saveSoftware(event) {
    const name = $("#newSoftwareName").val();

    let software = {
        name: name,
        type: event.data.type,
        sensors: [],
        actuators: [],
        software: [],
        renderedPosition: {}
    };

    if (event.data.type === 'timedCommand') {
        software['values'] = $("#newSoftwareValues").val();
        software['tsteps'] = $("#newSoftwareTSteps").val();

        //defaults
        if (software.values === "") { software.values = "[]" }
        if (software.tsteps === "") { software.tsteps = "[]" }
    }

    if (event.data.type === 'custom') {
        software['module'] = $("#newSoftwareModule").val()
    }


    if (event.data.new) {
        const height = cy.height();
        const width = cy.width();
        cy.add({
            group: 'nodes',
            data: {
                id: `software${name}`,
                label: name,
            },
            classes: 'software',
            renderedPosition: {
                x: width / 2,
                y: height / 2,
            },
        });
    } else {
        cy.$(`#software${event.data.name}`).data('id', `software${name}`)
        cy.$(`#software${event.data.name}`).data('label', name)

        software.sensors = JSAT.software[event.data.name].sensors
        software.actuators = JSAT.software[event.data.name].actuators
        software.software = JSAT.software[event.data.name].software

        delete JSAT.software[event.data.name]
    }
    software.renderedPosition = cy.$(`#software${name}`).renderedPosition();
    JSAT.software[name] = software;
    console.log(JSAT);
    $('#addSoftwareDiv').hide();
}

function editSoftware() {
    //remove all previously stored software specific inputs
    $('.software-input').remove();

    const name = this.data().label;
    const software = JSAT.software[name];

    $("#newSoftwareName").val(software.name);

    if (software.type === 'timedCommand') {
        addSoftwareTimedCommandInputs();
        $("#newSoftwareValues").val(software.values);
        $("#newSoftwareTSteps").val(software.tsteps);
    }

    if (software.type === 'custom') {
        addSoftwareCustomInputs();
        $("#newSoftwareModule").val(software.module)
    }

    $("#addSoftwareSaveButton").off();
    $("#addSoftwareSaveButton").on("click", { new: false, type: software.type, name: name }, saveSoftware)
    $("#addSoftwareDiv").show();

};

function addGravityConstantInputs() {
    $('#gravityTable tbody').append("<tr class = 'gravity-input'> \
            <td><label class='form-font'>value:</label><br></td> \
            <td><input id='newGravityValue' class='form-input' type='text' placeholder='-9.8'><br></td>\
        </tr>");
}

function clickAddGravityConstant() {
    //remove all old inputs
    $('.gravity-input').remove();
    //add software specific inputs
    addGravityConstantInputs();
    // bind actuator to save event, mark as new     
    $("#addGravitySaveButton").off()
    $("#addGravitySaveButton").on("click", { new: true, type: 'constant', name: '' }, saveGravity)
    //show the details div
    $('#addGravityDiv').show();
}

function clickAddGravityTwoBodyEarth() {
    //remove all old inputs
    $('.gravity-input').remove();
    $("#newGravityName").val("two_body_earth")
    $("#addGravitySaveButton").off()
    $("#addGravitySaveButton").on("click", { new: true, type: 'twoBodyEarth', name: '' }, saveGravity)
    $('#addGravityDiv').show();
}

function saveGravity(event) {
    const name = $("#newGravityName").val();

    let gravity = {
        name: name,
        type: event.data.type,
        renderedPosition: {},
    };

    if (event.data.type === 'constant') {
        gravity['value'] = $("#newGravityValue").val();

        //defaults
        if (gravity.value === "") { gravity.value = "-9.8" }
    }

    if (event.data.new) {
        const height = cy.height();
        const width = cy.width();
        cy.add({
            group: 'nodes',
            data: {
                id: `gravity${name}`,
                label: name,
            },
            classes: 'gravity',
            renderedPosition: {
                x: width / 2,
                y: height / 2,
            },
        });
    } else {
        cy.$(`#gravity${event.data.name}`).data('id', `gravity${name}`)
        cy.$(`#gravity${event.data.name}`).data('label', name)
        delete JSAT.gravity[event.data.name]
    }
    gravity.renderedPosition = cy.$(`gravity${name}`).renderedPosition();
    JSAT.gravity[name] = gravity;
    console.log(JSAT);
    $('#addGravityDiv').hide();
}

function editGravity() {
    //remove all previously stored software specific inputs
    $('.gravity-input').remove();

    const name = this.data().label;
    const gravity = JSAT.gravity[name];

    $("#newGravityName").val(gravity.name);

    if (software.type === 'constant') {
        addGravityConstantInputs();
        $("#newGravityValue").val(gravity.value);
    }

    $("#addGravitySaveButton").off();
    $("#addGravitySaveButton").on("click", { new: false, type: gravity.type, name: name }, saveGravity)
    $("#addGravityDiv").show();

};

function sendSimulationData() {
    const xhr = new XMLHttpRequest();
    xhr.open("POST", "/simulate");
    xhr.setRequestHeader("Content-Type", "application/json; charset=UTF-8");
    xhr.onreadystatechange = () => {
        // Call a function when the state changes.
        if (xhr.readyState === XMLHttpRequest.DONE && xhr.status === 200) {
            jsatConsole(`simulation completed in ${xhr.responseText} seconds!`)
            getSimFileNames()
        }
    };
    xhr.send(JSON.stringify(JSAT));
}

function getSimFileNames() {
    const xhr = new XMLHttpRequest();
    xhr.addEventListener("load", function () {
        let data = JSON.parse(this.responseText)
        // remove all options first...
        $("#simSelect").empty();
        $("#sim2Select").empty();
        //then reload all options        
        for (let i = 0; i < data.length; i++) {
            $("#simSelect").append($('<option>', {
                value: data[i].fileName,
                text: (data[i].fileName).concat(' ').concat(data[i].fileDate),
            }))
            $("#sim2Select").append($('<option>', {
                value: data[i].fileName,
                text: data[i].fileName.concat(' ').concat(data[i].fileDate),
            }))
        }
    })
    xhr.open("GET", "/simfiles", true);
    xhr.onreadystatechange = () => {
        // Call a function when the state changes.
        if (xhr.readyState === XMLHttpRequest.DONE && xhr.status === 200) {

        }
    };
    xhr.send();
}


function getCustomSoftware() {
    const xhr = new XMLHttpRequest();
    xhr.addEventListener("load", function () {
        let data = JSON.parse(this.responseText)
        console.log(data)
        for (let i = 0; i < data.length; i++) {
            $("#newSoftwareModule").append($('<option>', {
                value: data[i],
                text: data[i]
            }))
        }
    })
    xhr.open("GET", "/customsoftware", true);
    xhr.onreadystatechange = () => {
        // Call a function when the state changes.
        if (xhr.readyState === XMLHttpRequest.DONE && xhr.status === 200) {

        }
    };
    xhr.send();
}

function loadModels() {
    const xhr = new XMLHttpRequest();
    xhr.addEventListener("load", function () {
        let models = JSON.parse(this.responseText)

        //remove old buttons
        $('.loaded-model-button').remove();

        //make model buttons for each model
        let keys = Object.keys(models)
        for (let i = 0; i < keys.length; i++) {
            let name = keys[i];
            let btn = $('<button/>', {
                id: `${name}ModelButton`,
                html: name
            });
            btn.addClass('model-button');
            btn.addClass('loaded-model-button');
            //place button in div            
            $('#modelsLoaderDiv').append(btn);

            btn.on("click", function () {
                cy.add({
                    group: 'nodes',
                    data: {
                        id: name,
                        label: name
                    },
                    classes: 'model',
                    renderedPosition: {
                        x: 300,
                        y: 300,
                    },
                });
            });
        };

    });
    xhr.open("GET", "/loadmodels", true);
    xhr.send();
}

function getSimStates() {

    let selected = []
    $("#simSelect").find(":selected").each(function () { selected.push($(this).val()) })

    const xhr = new XMLHttpRequest();
    xhr.addEventListener("load", function () {
        const states = JSON.parse(xhr.responseText).states;
        // remove all options first...
        $("#xStateSelect").empty();
        $("#yStateSelect").empty();
        //then reload all options
        for (let i = 0; i < states.length; i++) {
            $("#xStateSelect").append($('<option>', {
                value: states[i],
                text: states[i],
            }))
            $("#yStateSelect").append($('<option>', {
                value: states[i],
                text: states[i],
            }))
        }
        $("#plotState").data("simOrFile", 0); //0 for Sim
    });
    xhr.open("POST", "/loadstates");
    xhr.send(JSON.stringify(selected));
}

$('#loadFileInput').on('change', function (e) {
    if (e.target.files[0]) {
        $("#chosenFileText").text(e.target.files[0].name)
        Papa.parse(e.target.files[0], {
            dynamicTyping: true,
            complete: function (results) {
                console.log(results)
                let states = results.data[0];

                if (states[0] === 'ASCII Report') {
                    //ITPS Report
                    //remove first 8 rows
                    results.data.splice(0, 8);
                    //add year in column 1 to sc time in column 2
                    if (results.data[0][0] === 'Year' && results.data[0][1] === 'S/C Time') {
                        for (let i = 1; i < results.data.length; i++) {
                            results.data[i][1] = results.data[i][0] + '-' + results.data[i][1];
                            results.data[i].splice(0, 1); //remove first column since its in column 2 now
                        }
                    }
                    results.data[0].splice(0,1); // remove year
                    states = results.data[0];
                }
                //console.log(states);
                const data = results.data.slice(1);
                // remove all options first...
                $("#xStateSelect").empty();
                $("#yStateSelect").empty();
                //then reload all options
                const reg = /(\d{4})-(\d{3})-(\d{2}):(\d{2}):(\S+)/ //YYYY-DDD-HH:MM:SS.
                for (let i = 0; i < states.length; i++) {
                    let values = data.map((x) => x[i]);
                    const tester = values[0];
                    if (typeof tester == "string") {
                        const match = tester.match(reg)
                        if (match) {
                            let new_values = [];
                            values.forEach(function (str) {
                                if (str == null || str == 'null-undefined') {
                                    new_values.push("")
                                } else {
                                    const each_match = str.match(reg)
                                    const Y = parseInt(each_match[1]);
                                    const D = parseInt(each_match[2]);
                                    const H = parseInt(each_match[3]);
                                    const M = parseInt(each_match[4]);
                                    const S = parseFloat(each_match[5]);
                                    const SI = Math.floor(S);
                                    const MS = (S-SI) * 1000;

                                    const date = new Date(Date.UTC(Y, 0, D, H, M, S, MS));
                                    new_values.push(date.toISOString());
                                }
                            });
                            values = new_values;
                        }
                    }

                    $("#xStateSelect").append($('<option>', { id: `xstate${i}`, text: states[i] }));
                    $("#yStateSelect").append($('<option>', { id: `ystate${i}`, text: states[i] }));

                    $(`#xstate${i}`).data("values", values);
                    $(`#ystate${i}`).data("values", values);

                }

            }
        });
        $("#plotState").data("simOrFile", 1); //1 for file
    };
});

function getSimOptions() {
    var tstart = $("#simStartTime").val();
    var tstop = $("#simStopTime").val();
    var dt = $("#simDt").val()
    if (dt == "") {
        dt = "nothing"
    }

    JSAT.sim.name = $("#simName").val();
    JSAT.sim.saveLocation = $("#simSaveLocation").val();
    JSAT.sim.nruns = $("#simNruns").val();
    JSAT.sim.tspan = `(${tstart},${tstop})`;
    JSAT.sim.dt = dt
}

function changeTab(evt) {

    // Get all elements with class="tabcontent" and hide them
    $(".tab-content").hide()
    // Get all elements with class="tablinks" and remove the class "active"
    $(".tab-link").removeClass("active-border");
    // highlight the tab button on top bar
    $(`#${evt.target.id}`).addClass("active-border")
    // show the associated div 
    switch (evt.target.id) {
        case "simTabButton":
            $("#simTab").show();
            break;
        case "plotTabButton":
            $("#plotTab").show();
            break;
        case "animTabButton":
            $("#animationTab").show();
            break;
    }
}

function deleteElements() {
    //get selected cy elements
    let elements = cy.elements()
    //remove bodies and joints from JSAT
    elements.forEach((ele) => {
        if (ele.selected()) {
            if (ele.group() === "nodes") {
                if (ele.hasClass('body')) {
                    delete JSAT.bodies[ele.data().label];
                }
                if (ele.hasClass("joint")) {
                    delete JSAT.joints[ele.data().label];
                }
                if (ele.hasClass("port")) {
                    if (ele.data().type === "in") {
                        delete JSAT.inports[ele.data().label];
                    }
                    if (ele.data().type === "out") {
                        delete JSAT.outports[ele.data().label];
                    }
                }
                if (ele.hasClass("gravity")) {
                    //delete from any bodies that have this gravity selected
                    let deletedGravityName = ele.data().label;
                    let bodyNames = Object.keys(JSAT.bodies);
                    for (let i = 0; i < bodyNames.length; i++) {
                        if (JSAT.bodies[bodyNames[i]].gravity.includes(deletedGravityName)) {
                            JSAT.bodies[bodyNames[i]].gravity = JSAT.bodies[bodyNames[i]].gravity.filter(e => e !== deletedGravityName);
                        }
                    }

                    if (JSAT.base.gravity.includes(deletedGravityName)) {
                        JSAT.base.gravity = JSAT.base.gravity.filter(e => e !== deletedGravityName);
                    }

                    delete JSAT.gravity[deletedGravityName];
                }

                if (ele.hasClass("actuator")) {
                    //delete from any bodies that have this actuator
                    let deletedActuator = ele.data().label;
                    let bodyNames = Object.keys(JSAT.bodies);
                    for (let i = 0; i < bodyNames.length; i++) {
                        if (JSAT.bodies[bodyNames[i]].actuators.includes(deletedActuator)) {
                            JSAT.bodies[bodyNames[i]].actuators = JSAT.bodies[bodyNames[i]].actuators.filter(e => e !== deletedActuator);
                        }
                    }

                    delete JSAT.actuators[deletedActuator];
                }
            }
            //remove from canvas
            ele.remove();
        }
    })

    console.log(JSAT)
}

function clickSaveScenario() {
    $("#newElementName").val("");
    $("#addElementSaveButton").off();
    $("#addElementSaveButton").on("click", saveScenario);
    $("#nameOnlyDiv").show();
}

function saveScenario() {
    const scenario_name = $("#newElementName").val();
    const xhr = new XMLHttpRequest();
    xhr.open("POST", "/savescenario");
    xhr.setRequestHeader("Content-Type", "application/json; charset=UTF-8");
    xhr.onreadystatechange = () => {
        // Call a function when the state changes.
        if (xhr.readyState === XMLHttpRequest.DONE && xhr.status === 200) {
            jsatConsole(`scenario saved`)
            $("#nameOnlyDiv").hide();
            getScenarios()
        }
    };
    let message = {
        name: scenario_name,
        scenario: JSAT
    }
    xhr.send(JSON.stringify(message));
}

function getScenarios() {
    const xhr = new XMLHttpRequest();
    xhr.addEventListener("load", function () {
        let scenarios = JSON.parse(this.responseText)
        //remove old buttons
        $('.loaded-scenario-button').remove();
        //make scenario buttons for each model        
        for (let i = 0; i < scenarios.length; i++) {
            let name = scenarios[i];
            let btn = $('<button/>', {
                id: `${name}ScenarioButton`,
                html: name
            });
            btn.addClass('model-button');
            btn.addClass('loaded-scenario-button');

            btn.on('click', { name: name }, loadScenario)
            //place button in div            
            $('#scenariosLoaderDiv').append(btn);
        }

    });
    xhr.open("GET", "/getscenarios", true);
    xhr.send();
}

function loadScenario(event) {
    const name = event.data.name;
    const xhr = new XMLHttpRequest();
    xhr.open("POST", "/loadscenario", true);
    xhr.setRequestHeader("Content-Type", "application/json; charset=UTF-8");
    xhr.addEventListener("load", function () {
        const response = JSON.parse(this.responseText)
        const jsat = response.scenario

        cy.elements().remove();

        //create the base

        const base = jsat.base;
        cy.add({
            group: 'nodes',
            data: {
                id: 'base',
                label: 'base',
            },
            classes: 'base',
            renderedPosition: base.renderedPosition,
        });

        //create all the bodies

        const bodies = Object.keys(jsat.bodies);
        for (let i = 0; i < bodies.length; i++) {
            let body = jsat.bodies[bodies[i]];
            cy.add({
                group: 'nodes',
                data: {
                    id: `body${body.name}`,
                    label: body.name,
                },
                classes: 'body',
                renderedPosition: body.renderedPosition,
            });
        }
        //create all the joints
        const joints = Object.keys(jsat.joints);
        for (let i = 0; i < joints.length; i++) {
            let joint = jsat.joints[joints[i]];
            cy.add({
                group: 'nodes',
                data: {
                    id: `joint${joint.name}`,
                    label: joint.name,
                },
                classes: 'joint',
                renderedPosition: joint.renderedPosition,
            });
        }
        //create all the actuators
        const actuators = Object.keys(jsat.actuators);
        for (let i = 0; i < actuators.length; i++) {
            let actuator = jsat.actuators[actuators[i]];
            cy.add({
                group: 'nodes',
                data: {
                    id: `actuator${actuator.name}`,
                    label: actuator.name,
                },
                classes: 'actuator',
                renderedPosition: actuator.renderedPosition,
            });
        }
        //create all the software
        const softwares = Object.keys(jsat.software);
        for (let i = 0; i < softwares.length; i++) {
            let software = jsat.software[softwares[i]];
            cy.add({
                group: 'nodes',
                data: {
                    id: `software${software.name}`,
                    label: software.name,
                },
                classes: 'software',
                renderedPosition: software.renderedPosition,
            });
        }
        //create all the sensors
        const sensors = Object.keys(jsat.sensors);
        for (let i = 0; i < sensors.length; i++) {
            let sensor = jsat.sensors[sensors[i]];
            cy.add({
                group: 'nodes',
                data: {
                    id: `sensor${sensor.name}`,
                    label: sensor.name,
                },
                classes: 'sensor',
                renderedPosition: sensor.renderedPosition,
            });
        }
        //create all the gravity
        const gravitys = Object.keys(jsat.gravity);
        for (let i = 0; i < gravitys.length; i++) {
            let gravity = jsat.gravity[gravitys[i]];
            cy.add({
                group: 'nodes',
                data: {
                    id: `gravity${gravity.name}`,
                    label: gravity.name,
                },
                classes: 'gravity',
                renderedPosition: gravity.renderedPosition,
            });
        }

        //make all connections
        if (base.gravity.length > 0) {
            for (let i = 0; i < base.gravity.length; i++) {
                cy.add({
                    group: 'edges',
                    data: { source: `gravity${base.gravity[i]}`, target: `base` }
                })
            }
        }

        for (let i = 0; i < joints.length; i++) {
            let joint = jsat.joints[joints[i]];
            if (joint.predecessor == 'base' || joint.predecessor == 'earth') {
                cy.add({
                    group: 'edges',
                    data: { source: `base`, target: `joint${joint.name}` }
                })
            } else {
                cy.add({
                    group: 'edges',
                    data: { source: `body${joint.predecessor}`, target: `joint${joint.name}` }
                })
            }
            if (joint.successor == 'base') {
                cy.add({
                    group: 'edges',
                    data: { source: `joint${joint.name}`, target: 'base' }
                })
            } else {
                cy.add({
                    group: 'edges',
                    data: { source: `joint${joint.name}`, target: `body${joint.successor}` }
                })
            }

        }

        for (let i = 0; i < bodies.length; i++) {
            let body = jsat.bodies[bodies[i]];
            for (let j = 0; j < body.sensors.length; j++) {
                let sensor = body.sensors[j]
                cy.add({
                    group: 'edges',
                    data: { source: `sensor${sensor}`, target: `body${body.name}` }
                })
            }

            for (let j = 0; j < body.actuators.length; j++) {
                let actuator = body.actuators[j]
                cy.add({
                    group: 'edges',
                    data: { source: `actuator${actuator}`, target: `body${body.name}` }
                })
            }

            for (let j = 0; j < body.gravity.length; j++) {
                let gravity = body.gravity[j]
                cy.add({
                    group: 'edges',
                    data: { source: `gravity${gravity}`, target: `body${body.name}` }
                })
            }

            for (let j = 0; j < body.environments.length; j++) {
                let environment = body.environments[j]
                cy.add({
                    group: 'edges',
                    data: { source: `environment${environment}`, target: `body${body.name}` }
                })
            }
        }

        for (let i = 0; i < softwares.length; i++) {
            let software = jsat.software[softwares[i]];
            for (let j = 0; j < software.actuators.length; j++) {
                let actuator = software.actuators[j]
                cy.add({
                    group: 'edges',
                    data: { source: `software${software.name}`, target: `actuator${actuator}` }
                })
            }

            for (let j = 0; j < software.sensors.length; j++) {
                let sensor = software.sensors[j]
                cy.add({
                    group: 'edges',
                    data: { source: `sensor${sensor}`, target: `software${software.name}` }
                })
            }
        }
        //TODO: set sim options


        cy.center()
        JSAT = jsat;
        console.log(JSAT)

    });
    xhr.send(JSON.stringify({ name: name }));
}
