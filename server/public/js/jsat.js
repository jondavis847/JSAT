import 'https://cdn.jsdelivr.net/npm/jquery@3.7.1/dist/jquery.min.js';
import 'https://cdn.jsdelivr.net/npm/jquery-ui@1.13.2/dist/jquery-ui.min.js';
import cytoscape from 'https://cdn.jsdelivr.net/npm/cytoscape@3.27.0/+esm';
import edgehandles from 'https://cdn.jsdelivr.net/npm/cytoscape-edgehandles@4.0.1/+esm';
cytoscape.use(edgehandles);
import "https://cdn.jsdelivr.net/npm/plotly.js/dist/plotly.min.js";
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export let JSAT = {
    bodies: {},
    joints: {},
    sim: {
        name: "",
        nruns: 0,
        tspan: "(0,10)",
        saveLocation: ""
    },
};

function jsatConsole(msg) {
    $("#consoleLog").val($("#consoleLog").val() + msg);
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
//on click
$("#simTabButton").on("click", changeTab);
$("#plotTabButton").on("click", changeTab);
$("#animTabButton").on("click", changeTab);
$("#simButton").on("click", sendSimulationData);
$("#addBodyCancelButton").on("click", () => { $("#addBodyDiv").hide() });
$("#addJointCancelButton").on("click", () => { $("#addJointDiv").hide() });
$("#loadModelButton").on("click", loadModel);
$("#loadSimStates").on("click", getSimStates);
$("#plotState").on("click", plotStateData);
$("#animateBtn").on("click", makeAnimation);
$("#basesButton").on("click", () => { $("#basesLoaderDiv").show() });
$("#basesBackButton").on("click", () => { $("#basesLoaderDiv").hide() });
$("#bodiesButton").on("click", () => { $("#bodiesLoaderDiv").show() });
$("#bodyBackButton").on("click", () => { $("#bodiesLoaderDiv").hide() });
$("#jointsButton").on("click", () => { $("#jointsLoaderDiv").show() });
$("#jointBackButton").on("click", () => { $("#jointsLoaderDiv").hide() });
$("#boxButton").on('click', clickAddBoxBody);
$("#cylinderButton").on('click', clickAddCylinderBody);
$('#baseButton').on('click', addBase);
$('#revoluteButton').on('click', clickAddRevoluteJoint);
$('#drawModeBtn').on('click',toggleDrawMode);

getSimFileNames();
loadModels();

let rp;
var cy = cytoscape({
    container: $("#cyCanvasDiv"),
    style: [
        {
            selector: '*',
            style: {
                'font-size': 12,
                'text-valign': 'center',
                'text-halign': 'center',
                'label': 'data(label)'
            }
        },
        {
            selector: '.base',
            style: {
                'shape': 'round-triangle',
                'background-color': 'whitesmoke',
                'width': 50,
                'height': 50,
            }
        },
        {
            selector: '.body',
            style: {
                'shape': 'round-rectangle',
                'background-color': 'aquamarine',
                'width': 50,
                'height': 50,
            }
        },
        {
            selector: '.joint',
            style: {
                'shape': 'ellipse',
                'background-color': 'aqua',
                'width': 50,
                'height': 50,
            }
        },
        {
            selector: 'edge',
            style: {
                'width': 3,
                'line-color': '#ccc',
                'target-arrow-color': '#ccc',
                'target-arrow-shape': 'triangle',
                'curve-style': 'bezier'
            }
        },
        { // just to suppress a warning with edgehandles https://github.com/cytoscape/cytoscape.js-edgehandles/issues/119
            //didnt work
            selector: '.eh-ghost-node',
            style: {
                'label': ''
            }
        }
    ],
});
// the default values of each option are outlined below:
/*
let ehdefaults = {
    canConnect: function (sourceNode, targetNode) {
        // whether an edge can be created between source and target
        return !sourceNode.same(targetNode); // e.g. disallow loops
    },
    edgeParams: function (sourceNode, targetNode) {
        // for edges between the specified source and target
        // return element object to be passed to cy.add() for edge
        return {};
    },
    //hoverDelay: 150, // time spent hovering over a target node before it is considered selected
    snap: false, // when enabled, the edge can be drawn by just moving close to a target node (can be confusing on compound graphs)
    //snapThreshold: 50, // the target node must be less than or equal to this many pixels away from the cursor/finger
    //snapFrequency: 15, // the number of times per second (Hz) that snap checks done (lower is less expensive)
    //noEdgeEventsInDraw: true, // set events:no to edges during draws, prevents mouseouts on compounds
    //disableBrowserGestures: true // during an edge drawing gesture, disable browser gestures such as two-finger trackpad swipe and pinch-to-zoom
};
*/
let eh = cy.edgehandles({ snap: false });
//cy.autoungrabify(false);

/*
$(document).on("keydown", (event) => {
    if (event.keyCode === 16) {
        eh.enableDrawMode();
    }
});
$(document).on("keyup", (event) => {
    if (event.keyCode === 16) {
        eh.disableDrawMode();
    }
});
*/
cy.on('ehcomplete', (evt, src, tar, edge) => {
    console.log(JSAT)

    //set if predecessor
    if (src.classes().includes("body")) {
        if (tar.classes().includes("joint")) {
            const source_id = src.data().label;
            const target_id = tar.data().label;
            JSAT.joints[target_id]["predecessor"] = source_id;
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
    cy.nodes().forEach(function (ele) {
        ele.grabify();
        console.log(ele.grabbable());
    })
    //cy.$('node').grabify(true); //for some reason nodes become ungrabbable after connecting an edgehandle
});


cy.on('dbltap', '.body', editBody)
cy.on('dbltap', '.joint', editJoint)

/*
cy.on('tap', 'node', function (evt) {
    evt.target.select();
    console.log(cy.$('node').selected())
});
*/

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
    if (cy.$('.base').length > 0) {
        jsatConsole('\ncant have more than 1 base!')
    } else {
        cy.add({
            group: 'nodes',
            data: {
                id: 'base',
                label: 'base'
            },
            classes: 'base',
            renderedPosition: {
                x: 300,
                y: 300,
            },
        });
    }
}

function addBodyBoxInputs() {
    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font " for="newBodyXLength">x length:</label><br></td> \
            <td><input id="newBodyXLength" class="form-input" type="text" name="newXBodyLength" placeholder="x length"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font " for="newBodyYLength">y length:</label><br></td> \
            <td><input id="newBodyYLength" class="form-input" type="text" name="newYBodyLength" placeholder="y length"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font " for="newBodyZLength">z length:</label><br></td> \
            <td><input id="newBodyZLength" class="form-input" type="text" name="newZBodyLength" placeholder="z length"><br></td>\
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
            <td><input id="newBodyRadiusTop" class="form-input" type="text" name="newBodyRadiusTop" placeholder="default 1"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font " for="newBodyRadiusBottom">radius bottom:</label><br></td> \
            <td><input id="newBodyRadiusBottom" class="form-input" type="text" name="newBodyRadiusBottom" placeholder="default 1"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font " for="newBodyHeight">height:</label><br></td> \
            <td><input id="newBodyHeight" class="form-input" type="text" name="newBodyHeight" placeholder="default 1"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font">radial segments:</label><br></td> \
            <td><input id="newBodyRadialSegments" class="form-input" type="text" placeholder="default 32"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font">height segments:</label><br></td> \
            <td><input id="newBodyHeightSegments" class="form-input" type="text" placeholder="default 1"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font">open ended:</label><br></td> \
            <td><input id="newBodyOpenEnded" class="form-input" type="text" placeholder="default capped"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font">theta start:</label><br></td> \
            <td><input id="newBodyThetaStart" class="form-input" type="text" placeholder="default 0"><br></td>\
        </tr>');

    $('#bodyTable tbody').append('<tr class = "geometry-input"> \
            <td><label class="form-font">theta length:</label><br></td> \
            <td><input id="newBodyThetaLength" class="form-input" type="text" placeholder="default 2pi"><br></td>\
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
        mass: $("#newBodyMass").val(),
        cm: $("#newBodyCm").val(),
        ixx: $("#newBodyIxx").val(),
        iyy: $("#newBodyIyy").val(),
        izz: $("#newBodyIzz").val(),
        ixy: $("#newBodyIxy").val(),
        ixz: $("#newBodyIxz").val(),
        iyz: $("#newBodyIyz").val(),
        geometry: event.data.geometry,
        material: $("#newBodyMaterial").val(),
        color: $("#newBodyColor").val(),         
    };

    //defaults
    if (body.mass === "") { body.mass = "1" }
    if (body.cm === "") { body.cm = "zeros(3)" }
    if (body.ixx === "") { body.ixx = "1" }
    if (body.iyy === "") { body.iyy = "1" }
    if (body.izz === "") { body.izz = "1" }
    if (body.ixy === "") { body.ixy = "0" }
    if (body.ixz === "") { body.ixz = "0" }
    if (body.iyz === "") { body.iyz = "0" }
    if (body.material === "") { body.material = "basic" }
    if (body.color === "") { body.color = "green" }

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
    if (event.data.new) {
        const name = $("#newBodyName").val();

        cy.add({
            group: 'nodes',
            data: {
                id: `body${name}`,
                label: name,
            },
            classes: 'body',
            renderedPosition: {
                x: 300,
                y: 300,
            },
        });
    } else {
        delete JSAT.bodies[event.data.name]
    }
    
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
    $("#newBodyMass").val(body.mass);
    $("#newBodyCm").val(body.cm);
    $("#newBodyIxx").val(body.ixx);
    $("#newBodyIyy").val(body.iyy);
    $("#newBodyIzz").val(body.izz);
    $("#newBodyIxy").val(body.ixy);
    $("#newBodyIxz").val(body.ixz);
    $("#newBodyIyz").val(body.iyz);
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

function saveJoint(event) {
    const name = $("#newJointName").val();

    let joint = {
        name: name,
        type: event.data.type,                
        FpRho: $("#newJointFpRho").val(),
        FpPhi: $("#newJointFpPhi").val(),
        FsRho: $("#newJointFsRho").val(),
        FsPhi: $("#newJointFsPhi").val(),
        predecessor: "undef", //defined after connection
        successor: "undef",  //defined after connection
    };

    //defaults
    if (joint.FpRho === "") { joint.FpRho = "zeros(3)" }
    if (joint.FsRho === "") { joint.FsRho = "zeros(3)" }
    if (joint.FpPhi === "") { joint.FpPhi = "I(3)" }
    if (joint.FsPhi === "") { joint.FsPhi = "I(3)" }
    

    if (event.data.type === 'revolute') {
        joint['theta'] = $("#newJointTheta").val();
        joint['omega'] = $("#newJointOmega").val();

        //defaults
        if (joint.theta === "") { joint.theta = "0" }
        if (joint.omega === "") { joint.omega = "0" }
    }
    
    if (event.data.new) {
        cy.add({
            group: 'nodes',
            data: {
                id: `joint${name}`,
                label: name,
            },
            classes: 'joint',
            renderedPosition: {
                x: 300,
                y: 300,
            },
        });        
    } else {
        delete JSAT.joints[event.data.name]    
    }
    
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
    }

    $("#addJointDiv").show();
};

function sendSimulationData() {
    const xhr = new XMLHttpRequest();
    xhr.open("POST", "/simulate");
    xhr.setRequestHeader("Content-Type", "application/json; charset=UTF-8");
    xhr.onreadystatechange = () => {
        // Call a function when the state changes.
        if (xhr.readyState === XMLHttpRequest.DONE && xhr.status === 200) {
            jsatConsole(`\nsimulation completed in ${xhr.responseText} seconds!`)
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

function loadModels() {
    /*
    const xhr = new XMLHttpRequest();
    xhr.addEventListener("load", function () {
        MODELS = JSON.parse(this.responseText)

        //make model buttons for each model
        let keys = Object.keys(MODELS)
        for (let i = 0; i < keys.length; i++) {
            let btn = $('<button/>', {
                id: `${keys[i]}ModelButton`,
                html: keys[i]
            });
            btn.addClass("model-buttons");
            //place button in div            
            $("#modelLoaderDiv").append(btn);

            btn.on("click", function () {
                $(".model-buttons").removeClass("active-border")
                btn.toggleClass("active-border")
                console.log(keys[i])
                console.log(MODELS)
                console.log(MODELS[keys[i]])
                CURRENTMODEL = MODELS[keys[i]]
            });
        };
        
    });
    xhr.open("GET", "/loadmodels", true);
    xhr.send();
    */
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

    });
    xhr.open("POST", "/loadstates");
    xhr.send(JSON.stringify(selected));
}

function plotStateData() {

    let colormap = [
        [0, 255, 159],
        [0, 184, 255],
        [189, 0, 255],
        [255, 95, 31],
        [0, 30, 255],
        [234, 0, 217],
    ];

    let selectedSims = []
    $("#simSelect").find(":selected").each(function () { selectedSims.push($(this).val()) })
    let selectedXState = $("#xStateSelect").find(":selected").val();    
    let selectedYStates = []
    $("#yStateSelect").find(":selected").each(function () { selectedYStates.push($(this).val()) })
    const selectedStates  = [ ...new Set([selectedXState, ...selectedYStates])]
    console.log(selectedStates)
    const xhr = new XMLHttpRequest();
    xhr.addEventListener("load", function () {
        const simData = JSON.parse(xhr.responseText);
        let traces = [];
        let colorCtr = 0;        
        simData.forEach(function (sim) {
            selectedYStates.forEach(function (yState) {
                var putInLegend = true; //only 1 state per 1 sim in legend (collect runs)
                sim.runData.forEach(function (run, i) {                    
                    let x = run.colindex.names.indexOf(selectedXState)
                    let y = run.colindex.names.indexOf(yState)
                    traces.push({
                        x: run.columns[x],
                        y: run.columns[y],
                        type: 'scatter',
                        mode: 'lines',
                        name: `${sim.sim}_${yState}`,
                        showlegend: putInLegend,
                        line: {
                            color: `rgb(${colormap[colorCtr]})`,
                            width: 1
                        },
                        opacity: (i >= 1) ? 0.2 : 1,
                    })
                    putInLegend = false; //dont put more than 1 run in legend                        
                })
                colorCtr++;
                if (colorCtr >= colormap.length) {
                    colorCtr = 0
                }
            })

        })
        let layout = {
            plot_bgcolor: "rgb(35, 36, 36)",
            paper_bgcolor: "rgb(35, 36, 36)",
            font: {
                family: 'Courier New, monospace',
                size: 10,
                color: 'aliceblue'
            },
            title: selectedYStates.join(),
            showlegend: true,
            legend: {
                orientation: "h",
                bgcolor: 'rgba(0,0,0,0)' //transparent
            },
            xaxis: {
                title: selectedXState,
                gridcolor: "rgb(0,0,0)"
            },
            yaxis: {
                gridcolor: "rgb(0,0,0)"
            },
            margin: {
                l: 50,
                r: 50,
                b: 50,
                t: 50,
                pad: 4
            },
        }
        Plotly.newPlot("plotsDiv", traces, layout, { scrollZoom: true })
    });

    xhr.open("POST", "/plotstates");
    xhr.send(JSON.stringify({ sims: selectedSims, states: selectedStates }));
}


function getSimOptions() {
    var tstart = $("#simStartTime").val();
    var tstop = $("#simStopTime").val();

    JSAT.sim.name = $("#simName").val();
    JSAT.sim.saveLocation = $("#simSaveLocation").val();
    JSAT.sim.nruns = $("#simNruns").val();
    JSAT.sim.tspan = `(${tstart},${tstop})`;
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

function loadModel() {

    let nbodies = CURRENTMODEL.bodies.length

    for (let b = 0; b < nbodies; b++) {
        NEWBODY = true;
        CURRENTBODY = CURRENTMODEL.bodies[b];
        $(".model-buttons").removeClass("active-border");
        saveBody(CURRENTBODY);
    }
    console.log(CURRENTMODEL)
    let njoints = (CURRENTMODEL.joints).length

    for (let j = 0; j < njoints; j++) {
        NEWJOINT = true;
        CURRENTJOINT = CURRENTMODEL.joints[j];
        $(".model-buttons").removeClass("active-border");
        saveJoint(CURRENTJOINT);
    }

    CURRENTMODEL = {};
}





const savedSpacecraft = [
    {
        name: "fake_pace",
        sc: {
            body: {
                name: "body",
                ixx: "1000",
                iyy: "1000",
                izz: "1000",
                ixy: "0",
                ixz: "0",
                iyz: "0",
                q: "[0, 0, 0, 1]",
                w: "zeros(3)",
                r: "[-3.9292738554734, 5.71264013167723, 1.31199443874228]*1e6",
                v: "[84.5551344721184, 1749.4937756303016, -7311.912202797997]",
            },
            reactionwheels: [
                {
                    name: "rw1",
                    J: "0.25",
                    kt: "0.075",
                    a: "[1, 0, 0]",
                    w: "0",
                },
                {
                    name: "rw2",
                    J: "0.25",
                    kt: "0.075",
                    a: "[0, 1, 0]",
                    w: "0",
                },
                {
                    name: "rw3",
                    J: "0.25",
                    kt: "0.075",
                    a: "[0, 0, 1]",
                    w: "0",
                },
            ],
            thrusters: [
                {
                    name: "thr1",
                    F: "1",
                    r: "[1, 0, -1]",
                    R: "[0 1 0; 1 0 0; 0 0 1]"
                },
                {
                    name: "thr2",
                    F: "1",
                    r: "[-1, 0, 1]",
                    R: "[0 1 0; 1 0 0; 0 0 1]"
                },
                {
                    name: "thr3",
                    F: "1",
                    r: "[0, 1, -1]",
                    R: "[0 1 0; 1 0 0; 0 0 1]"
                },
                {
                    name: "thr4",
                    F: "1",
                    r: "[0, -1, -1]",
                    R: "[0 1 0; 1 0 0; 0 0 1]"
                },
            ],
            iru: {
                name: "iru",
                sigma: "0.01",
            },
            controller: {
                name: "controller"
            },
        }
    }
]

function makeAnimation() {
    let selectedSims = []
    $("#sim2Select").find(":selected").each(function () { selectedSims.push($(this).val()) });
    if (selectedSims.length === 0) {
        jsatError("!no sim selected - please select a sim")
    }
    if (selectedSims.length > 1) {
        jsatError("!too many sims selected - please select only 1 sim")
    }

    const xhr = new XMLHttpRequest();
    xhr.addEventListener("load", function () {
        const data = JSON.parse(xhr.responseText);
        const sys = data.sys;
        const animData = data.data;

        const animationDiv = document.getElementById("animationDiv")
        var renderer = new THREE.WebGLRenderer({ antialias: true, logarithmicDepthBuffer: true });
        let w = animationDiv.offsetWidth;
        let h = animationDiv.offsetHeight;
        renderer.setSize(w, h);
        animationDiv.appendChild(renderer.domElement);

        const scene = new THREE.Scene();
        const axesHelper = new THREE.AxesHelper(1);
        scene.add(axesHelper);
        const camera = new THREE.PerspectiveCamera(300, w / h, 1, 100);
        camera.position.set(0, 0, 10);
        camera.rotation.set(0, 0, Math.PI)
        //camera.up.set(0,1,0);


        //const controls = new OrbitControls(camera, renderer.domElement);
        //controls.update()

        const light = new THREE.AmbientLight('white', 1); // soft white light
        scene.add(light);

        //create bodys
        const body_keys = Object.keys(sys.bodies);
        for (let i = 0; i < body_keys.length; i++) {
            const body = sys.bodies[body_keys[i]];
            let geometry;
            if (body.geometry === 'box') {
                geometry = new THREE.BoxGeometry(
                    body.width,
                    body.height,
                    body.length
                );
            } else if (body.geometry === 'cylinder') {
                geometry = new THREE.CylinderGeometry(
                    body.radiusTop,
                    body.radiusBottom,
                    body.height,
                    body.radialSegments,
                    body.heightSegments,
                    body.openEnded,
                    body.thetaStart,
                    body.thetaLength
                );
                geometry.rotateX(Math.PI/2) //force cylinders to be z in the height direction
            }
            const material = new THREE.MeshBasicMaterial({ color: body.color });
            const mesh = new THREE.Mesh(geometry, material);
            mesh.name = body.name;

            let q1_index = animData.colindex.names.indexOf(`${body.name}_q_base[1]`);
            let q2_index = animData.colindex.names.indexOf(`${body.name}_q_base[2]`);
            let q3_index = animData.colindex.names.indexOf(`${body.name}_q_base[3]`);
            let q4_index = animData.colindex.names.indexOf(`${body.name}_q_base[4]`);
            let r1_index = animData.colindex.names.indexOf(`${body.name}_r_base[1]`);
            let r2_index = animData.colindex.names.indexOf(`${body.name}_r_base[2]`);
            let r3_index = animData.colindex.names.indexOf(`${body.name}_r_base[3]`);

            let this_data = {
                q1: animData.columns[q1_index],
                q2: animData.columns[q2_index],
                q3: animData.columns[q3_index],
                q4: animData.columns[q4_index],
                r1: animData.columns[r1_index],
                r2: animData.columns[r2_index],
                r3: animData.columns[r3_index],
            }
            mesh.userData = this_data;
            scene.add(mesh);
        }
        const clock = new THREE.Clock({ autoStart: false });
        //controls.update();


        /*
        let deltaSpacecraftCamera = new THREE.Vector3(0, 0, 300);
        controls.addEventListener("change", function () {
            deltaSpacecraftCamera.subVectors(camera.position, spacecraft.position);
        });
*/
        const time_index = animData.colindex.names.indexOf("t");
        const time_data = animData.columns[time_index];
        let t;
        const findTimeIndex = (data) => (data > t);
        let start_time;
        let sim_elapsed_time;
        let t0 = time_data[0];

        function animate() {
            requestAnimationFrame(animate);

            if (clock.running) {
                sim_elapsed_time = clock.getElapsedTime() - start_time;
                t = t0 + sim_elapsed_time;
                if (t > time_data[time_data.length - 1]) {
                    t = t0;
                    start_time = clock.getElapsedTime()
                }
            } else {
                t = t0;
                clock.start()
                start_time = 0
            }

            const i = time_data.findIndex(findTimeIndex);
            for (let b = 0; b < body_keys.length; b++) {
                let this_body = sys.bodies[body_keys[b]];
                let body = scene.getObjectByName(this_body.name);
                body.position.set(body.userData.r1[i], body.userData.r2[i], body.userData.r3[i]);
                body.quaternion.set(body.userData.q1[i], body.userData.q2[i], body.userData.q3[i], body.userData.q4[i]);
            }
            /*
            spacecraft.position.set(data.r1[i], data.r2[i], data.r3[i]);
            spacecraft.quaternion.set(data.q1[i], data.q2[i], data.q3[i], data.q4[i]);
            camera.position.set(data.r1[i] + deltaSpacecraftCamera.x, data.r2[i] + deltaSpacecraftCamera.y, data.r3[i] + deltaSpacecraftCamera.z)
            controls.target.copy(spacecraft.position);
            */
            renderer.render(scene, camera);

        }

        animate();

    });
    xhr.open("POST", "/animation");
    xhr.send(JSON.stringify({ sim: selectedSims[0], run: "run0" }));
    /*
            const rEarth = 6.3781e6;
            const i_earth = new THREE.TextureLoader().load('./images/earth.jpeg');
            const g_earth = new THREE.SphereGeometry(rEarth, 64, 64)
            const m_earth = new THREE.MeshPhongMaterial({ map: i_earth });
            const earth = new THREE.Mesh(g_earth, m_earth);
            earth.position.set(0, 0, 0);
            scene.add(earth);
    
            const light = new THREE.AmbientLight('white', 0.05); // soft white light
            scene.add(light);
    
            const sunlight = new THREE.DirectionalLight('white');
            sunlight.position.set(-1, 0, 0);
            scene.add(sunlight);
    
            const rSun = 1 * 696340000;
            const dSun = 151.39099 * 1e6 * 1e3; // 151 million km
            const g_sun = new THREE.SphereGeometry(rSun, 16, 16)
            const m_sun = new THREE.MeshBasicMaterial({ color: 'white' })
            const sun = new THREE.Mesh(g_sun, m_sun);
            sun.position.set(-dSun, 0, 0);
            scene.add(sun)
    
    
            const gltfLoader = new GLTFLoader();
            // Load a glTF resource
            let spacecraft;
    
            gltfLoader.load(
                // resource URL
                'images/PACE_simplified.gltf',
                // called when the resource is loaded
                function (gltf) {
    
                    spacecraft = gltf.scene;
    
                    spacecraft.children.forEach(function (mesh) {
                        mesh.material = new THREE.MeshPhongMaterial({ color: 'silver' });
                    });
    
                    //pace.position.set(data.r1[0],data.r2[0],data.r3[0]);
                    spacecraft.position.set(0, 0, rEarth + 700000)
                    spacecraft.quaternion.set(data.q1[0], data.q2[0], data.q3[0], data.q4[0]);
                    console.log(spacecraft)
    
                    scene.add(spacecraft);
                    renderer.render(scene, camera);
    
                },
                // called while loading is progressing
                function (xhr) {
    
                    //console.log((xhr.loaded / xhr.total * 100) + '% loaded');
    
                },
                // called when loading has errors
                function (error) {
    
                    console.log('An error happened');
    
                }
            );
    
            */



}