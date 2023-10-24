//import $ from "jquery";
//import * as Plotly from "plotly.js-dist"
//import * as THREE from 'three';
//import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
//import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

let JSAT = {
    bodies: {},
    joints: {},
    sim: {
        name: "",
        nruns: 0,
        tspan: "(0,10)",
        saveLocation: ""
    },
};

//true when adding a new body by clicking + body, false when editing a body by clicking body button
let NEWBODY = true;
//just used to know which body we're working with
let CURRENTBODY;

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

function init() {
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
    $("#addBodyButton").on("click", addBody);
    $("#addBodySaveButton").on("click", saveBody);
    $("#addBodyCancelButton").on("click", cancelBody);
    $("#loadModelButton").on("click", loadModel);
    $("#addJointButton").on("click", function () { $("#jointMenu").toggle() });
    $("#cancelJointButton").on("click", function () { $("#jointMenu").toggle() });
    $("#addJointCancelButton").on("click", cancelJointPopup);
    $("#loadSimStates").on("click", loadSimStates);
    $("#plotState").on("click", plotStateData);
    $("#animateBtn").on("click", makeAnimation);

    //on keyup
    enterClick($("#bodyName"), $("#addBodySaveButton"));
    enterClick($("#jointName"), $("#addJointSaveButton"));

    createBodyDetailsDiv();
    // loadModels();
    // getSimFileNames();
}
$(window).on("load", init);

/*
function sendSimulationData() {
    const socket = new WebSocket("ws://localhost:8081");
    jsatConsole("\nconnecting to jsat server...")
    socket.addEventListener("open", (event) => {
        jsatConsole("connected!\nsending simulation data")
        socket.send(JSON.stringify({ type: "simulationData", data: JSON.stringify(JSAT) }));
    });
    socket.addEventListener("message", (event) => {
        console.log(event)
        jsatConsole(event.data)
        getSimFileNames()
        socket.close()
    });

}
*/

function sendSimulationData() {
    const xhr = new XMLHttpRequest();
    xhr.open("GET", "/simulate", true);
    xhr.onreadystatechange = () => {
        // Call a function when the state changes.
        if (xhr.readyState === XMLHttpRequest.DONE && xhr.status === 200) {
            console.log("success")
        }
    };
    xhr.send();
}

function getSimFileNames() {
    const socket = new WebSocket("ws://localhost:8081");
    jsatConsole("\nconnecting to jsat server...")
    socket.addEventListener("open", (event) => {
        jsatConsole("connected!")
        socket.send(JSON.stringify({ type: "getSimFileNames", data: [] }));
    });
    socket.addEventListener("message", (event) => {
        jsatConsole("\nrecieved data from server")
        const simFileNames = JSON.parse(event.data).simFileNames;
        const simFileDates = JSON.parse(event.data).simFileDates;
        // remove all options first...
        $("#simSelect").empty();
        //then reload all options
        for (let i = 0; i < simFileNames.length; i++) {
            $("#simSelect").append($('<option>', {
                value: simFileNames[i],
                text: simFileNames[i].concat(' ').concat(simFileDates[i]),
            }))
            $("#sim2Select").append($('<option>', {
                value: simFileNames[i],
                text: simFileNames[i].concat(' ').concat(simFileDates[i]),
            }))
        }
        jsatConsole("\nupdated available simulations")
        socket.close()
    });
}

function loadSimStates() {

    let selected = []
    $("#simSelect").find(":selected").each(function () { selected.push($(this).val()) })
    const socket = new WebSocket("ws://localhost:8081");
    jsatConsole("\nconnecting to jsat server...")
    socket.addEventListener("open", (event) => {
        jsatConsole("connected!")
        socket.send(JSON.stringify({ type: "loadSimStates", data: selected }));
    });
    socket.addEventListener("message", (event) => {
        const states = JSON.parse(event.data).states;
        // remove all options first...
        $("#stateSelect").empty();
        //then reload all options
        for (let i = 0; i < states.length; i++) {
            $("#stateSelect").append($('<option>', {
                value: states[i],
                text: states[i],
            }))
        }
        jsatConsole("\nrecieved state data!")

        socket.close()
    });
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
    let selectedStates = []
    $("#stateSelect").find(":selected").each(function () { selectedStates.push($(this).val()) })


    const socket = new WebSocket("ws://localhost:8081");
    jsatConsole("\nconnecting to jsat server...")
    socket.addEventListener("open", (event) => {
        jsatConsole("connected!")
        socket.send(JSON.stringify({ type: "plot", data: JSON.stringify({ sims: selectedSims, states: selectedStates }) }));
    });
    socket.addEventListener("message", (event) => {
        const data = JSON.parse(event.data).data;
        const simData = JSON.parse(data);

        let traces = [];
        let colorCtr = 0;
        simData.forEach(function (sim) {
            selectedStates.forEach(function (state) {
                var putInLegend = true; //only 1 state per 1 sim in legend (collect runs)
                sim.runData.forEach(function (run, i) {
                    traces.push({
                        x: run.timestamp,
                        y: run[state],
                        type: 'scatter',
                        mode: 'lines',
                        name: `${sim.sim}_${state}`,
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
            title: selectedStates.join(),
            showlegend: true,
            legend: {
                orientation: "h",
                bgcolor: 'rgba(0,0,0,0)' //transparent
            },
            xaxis: {
                title: "t(s)",
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

        jsatConsole("\nrecieved state data!")
        socket.close()

    });
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

// called when user clicks +body, prompts user for the name, will save and add component on enter or click
function addBody() {
    NEWBODY = true;

    //reset body details inputs
    $("#body_name").val("");
    $("#body_mass").val("");
    $("#body_cm").val("");
    $("#body_ixx").val("");
    $("#body_iyy").val("");
    $("#body_izz").val("");
    $("#body_ixy").val("");
    $("#body_ixz").val("");
    $("#body_iyz").val("");

    $("#bodyDetailsDiv").show();
    $("#body_name").trigger("focus");
}



function createBodyDetailsDiv() {
    let body = {
        name: "",
        mass: "",
        cm: "",
        ixx: "",
        iyy: "",
        izz: "",
        ixy: "",
        ixz: "",
        iyz: "",
    };

    $("#bodyDetailsDiv").append(tableFromObject(body, "body"));
    const saveBtn = $("<button>/", {
        id: "bodyDetailsSaveBtn",
        html: "save"
    });

    saveBtn.addClass("save-body-details-button");
    saveBtn.on("click", saveBody)

    $("#bodyDetailsDiv").append(saveBtn);

    const cancelBtn = $("<button>/", {
        id: "bodyDetailsCancelBtn",
        html: "cancel"
    });
    cancelBtn.addClass("cancel-body-details-button");
    cancelBtn.on("click", function () {
        $("#bodyDetailsDiv").hide();
    });
    $("#bodyDetailsDiv").append(cancelBtn);
}

function editBody() {
    NEWBODY = false;
    CURRENTBODY = this;
    $("#body_name").val(JSAT.bodies[this].name);
    $("#body_mass").val(JSAT.bodies[this].mass);
    $("#body_cm").val(JSAT.bodies[this].cm);
    $("#body_ixx").val(JSAT.bodies[this].ixx);
    $("#body_iyy").val(JSAT.bodies[this].iyy);
    $("#body_izz").val(JSAT.bodies[this].izz);
    $("#body_ixy").val(JSAT.bodies[this].ixy);
    $("#body_ixz").val(JSAT.bodies[this].ixz);
    $("#body_iyz").val(JSAT.bodies[this].iyz);
    $("#bodyDetailsDiv").show();
};




// called when user saves the +body or edit body prompt
function saveBody() {
    const body = {
        name: $("#body_name").val(),
        mass: $("#body_mass").val(),
        cm: $("#body_cm").val(),
        ixx: $("#body_ixx").val(),
        iyy: $("#body_iyy").val(),
        izz: $("#body_izz").val(),
        ixy: $("#body_ixy").val(),
        ixz: $("#body_ixz").val(),
        iyz: $("#body_iyz").val(),
    };

    // throw error and return if any properties arent set        
    for (const [key, value] of Object.entries(body)) {
        if (value === "") {
            jsatConsole("\nall fields of body are required to have a value!")
            return;
        }
    }

    //throw an error if a body with that name already exists
    for (const [key, value] of Object.entries(JSAT.bodies)) {
        if (key === body.name) {
            jsatConsole("\nbody with that name already exists!")
            return;
        }
    }

    if (!NEWBODY) {
        delete JSAT.bodies[CURRENTBODY]
        $(`#${CURRENTBODY}BodyButton`).html(body.name)
    }
    JSAT.bodies[body.name] = body;

    if (NEWBODY) {
        //create new body button
        let btn = $('<button/>', {
            id: `${body.name}BodyButton`,
            html: body.name
        })
        btn.addClass("body-buttons")
        //place button in div
        $("#bodyBuilderDiv").append(btn);

        btn.on("click", editBody.bind(body.name))
    }
    $("#bodyDetailsDiv").hide();
    console.log(JSAT.bodies)
};

function addTableInput(table, name, attr, value, obj) {
    var tr = document.createElement("tr");

    //create label    
    var td1 = document.createElement("td");
    var l = document.createElement("label");
    l.setAttribute('for', `${obj}_${name}`);
    l.innerHTML = name;
    l.classList.add("form-font");
    var brl = document.createElement("br");
    td1.appendChild(l);
    td1.appendChild(brl);

    //create input    
    var td2 = document.createElement("td2");
    var i = document.createElement("input");
    i.setAttribute('type', "text");
    i.id = `${obj}_${name}`;
    i.setAttribute(attr, value);
    i.classList.add("form-input");
    var bri = document.createElement("br");
    td2.appendChild(i);
    td2.appendChild(bri);

    //append to table
    tr.appendChild(td1);
    tr.appendChild(td2);
    table.appendChild(tr);

    //add onblur event caller to update global SC
    //i.onblur = function () {
    //  COMPONENT[name] = i.value;
    //if this is name property, update the button text
    //if (name === "name") {
    //  COMPONENT_BUTTON.innerText = i.value;
    //}
    // }
}

function addComponent(componentType) {

    const name = $("#componentName").val();
    //create new component button
    let btn = $("<button/>", {
        id: `${SPACECRAFT.name}_${name}_btn`,
        html: name
    })
    btn.addClass("component-buttons")

    //add button to current spacecraft div
    SPACECRAFT_DIV.append(btn);
    //hide the popup
    $("#componentPopup").hide();
    //deactivate button
    $("#addComponentButton").removeClass("active-border");
    //reset input field
    $("#componentName").val(""); // make placeholder?

    //create div for new component
    const divId = `${SPACECRAFT.name}_${name}_div`
    let newDiv = $("<div>", {
        id: divId
    });
    newDiv.addClass("component-details-form-div");

    //add component to global SC
    switch (componentType) {
        case component.body:
            var tmp = {
                name: `${SPACECRAFT.name}_${name}`,
                ixx: "",
                iyy: "",
                izz: "",
                ixy: "",
                ixz: "",
                iyz: "",
                q: "",
                w: "",
                r: "",
                v: "",
            }
            SPACECRAFT.body = tmp;
            break;
        case component.reactionWheel:
            var tmp = {
                name: `${SPACECRAFT.name}_${name}`,
                J: "",
                kt: "",
                a: "",
                w: "",
            }
            SPACECRAFT.reactionWheels.push(tmp);
            break;
        case component.thruster:
            var tmp = {
                name: `${SPACECRAFT.name}_${name}`,
                F: "",
                r: "",
                R: "",
            }
            SPACECRAFT.thrusters.push(tmp);
            break;
        case component.iru:
            var tmp = {
                name: `${SPACECRAFT.name}_${name}`,
                sigma: "",
            }
            SPACECRAFT.iru = tmp;
            break;
        case component.controller:
            var tmp = {
                name: `${SPACECRAFT.name}_${name}`,
            }
            SPACECRAFT.controller = tmp;
            break;
    }
    btn.component = tmp;
    let t = tableFromObject(tmp);
    newDiv.append(t); //append table made in add<Component>()

    const c = $("<button>/", {
        html: "close"
    });
    c.addClass("save-component-details-button");

    c.on("click", function () {
        newDiv.hide();
        btn.removeClass("active-border");
        btn.addClass("not-active-border");
    });
    newDiv.append(c);

    $("#componentDetailsDiv").append(newDiv);

    //change the name field to the current name
    $(`#${SPACECRAFT.name}_${name}_name`).val(name);

    //onclick to switch component divs between spacecraft
    btn.on("click", function () {
        $(".component-details-form-div").hide();
        newDiv.show();
        //deactivate any currently active component
        $(".component-buttons").removeClass("active-border");
        $(".component-buttons").addClass("not-active-border");
        //activate new button
        btn.removeClass("not-active-border");
        btn.addClass("active-border");

        //set global component
        COMPONENT = btn.component;
        COMPONENT_BUTTON = btn;
    });

    btn.trigger("click");
}

function tableFromObject(object, id_name) {
    var t = document.createElement("table");
    t.classList.add("table");

    const k = Object.keys(object);
    for (let i = 0; i < k.length; i++) {
        addTableInput(t, k[i], 'placeholder', `enter ${k[i]}`, id_name); //change body to enum    
    }
    return t;
}




// just loads the options from savedSpacecraft when the window loads and adds it to the loadselect
function loadModels() {
    //for (let i = 0; i < savedSpacecraft.length; i++) {
    //   $("#spacecraftLoaderSelect").append($('<option>', {
    //      value: i,
    //     text: savedSpacecraft[i].name
    // }))
    // }
}

//takes fields from savedSpacecraft components and populates thier component fields when loaded
function populateInputFields(comp) {
    const keys = Object.keys(comp);
    keys.forEach(function (k) {
        //focus first so that onblur will populate JSAT.sc
        $(`#${SPACECRAFT.name}_${comp.name}_${k}`).trigger("focus");
        //replace input field text with the savedSpacecraft values         
        $(`#${SPACECRAFT.name}_${comp.name}_${k}`).val(comp[k]);
        //blur the field so that it updates SPACECRAFT and JSAT.sc
        $(`#${SPACECRAFT.name}_${comp.name}_${k}`).trigger("blur");
    })
}

//actually loads the selected savedSpacecraft into the console and makes buttons
function loadModel() {

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
    let data;
    let selectedSims = []
    $("#sim2Select").find(":selected").each(function () { selectedSims.push($(this).val()) });
    if (selectedSims.length === 0) {
        jsatError("!no sim selected - please select a sim")
    }
    if (selectedSims.length > 1) {
        jsatError("!too many sims selected - please select only 1 sim")
    }
    const socket = new WebSocket("ws://localhost:8081");
    socket.addEventListener("open", (event) => {
        jsatConsole("connected!")
        socket.send(JSON.stringify({ type: "animate", data: JSON.stringify({ sims: selectedSims[0] }) }));
    });

    socket.addEventListener("message", (event) => {
        data = JSON.parse(event.data);
        // THREE stuff
        const animationDiv = document.getElementById("animationDiv")
        var renderer = new THREE.WebGLRenderer({ antialias: true, logarithmicDepthBuffer: true });
        let w = animationDiv.offsetWidth;
        let h = animationDiv.offsetHeight;
        renderer.setSize(w, h);
        animationDiv.appendChild(renderer.domElement);

        const scene = new THREE.Scene();

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


        const camera = new THREE.PerspectiveCamera(75, w / h, 1, 2 * dSun);
        //camera.position.set(new THREE.Vector3(data.r1[0] - 5,data.r2[0] + 5,data.r3[0] + 5));
        camera.position.set(0, 0, rEarth + 700000 + 300);
        //camera.lookAt(new THREE.Vector3(0,0,0));


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

        const clock = new THREE.Clock({ autoStart: false });
        //controls.update();

        const controls = new OrbitControls(camera, renderer.domElement);
        let deltaSpacecraftCamera = new THREE.Vector3(0, 0, 300);
        controls.addEventListener("change", function () {
            deltaSpacecraftCamera.subVectors(camera.position, spacecraft.position);
        });

        function animate() {
            requestAnimationFrame(animate);
            let t;

            if (clock.running) {
                t = clock.getElapsedTime()
            } else {
                t = 0;
                clock.start()
            }

            const findTimeIndex = (data) => (data > t);
            const i = data.t.findIndex(findTimeIndex);
            spacecraft.position.set(data.r1[i], data.r2[i], data.r3[i]);
            spacecraft.quaternion.set(data.q1[i], data.q2[i], data.q3[i], data.q4[i]);
            camera.position.set(data.r1[i] + deltaSpacecraftCamera.x, data.r2[i] + deltaSpacecraftCamera.y, data.r3[i] + deltaSpacecraftCamera.z)
            controls.target.copy(spacecraft.position);
            renderer.render(scene, camera);
        }

        animate();

    });


}
