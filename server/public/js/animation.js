import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TrackballControls } from 'three/addons/controls/TrackballControls.js';

export let ANIMATION_ID = null; //used to cancel animations
export let PLAYBACK_SPEED = 1.0;

export function makeAnimation() {

    // cancel any ongoing animation
    if (ANIMATION_ID !== null) {
        cancelAnimationFrame(ANIMATION_ID)
    }

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
        console.log(sys)
        const animData = data.data;

        const animationDiv = document.getElementById("animationDiv")
        var renderer = new THREE.WebGLRenderer({ antialias: true, logarithmicDepthBuffer: true });
        //renderer.shadowMap.enabled = true; //doesnt seem to be working, earth not casting shadow on obs
        renderer.shadowMap.type = THREE.BasicShadowMap;

        let w = animationDiv.offsetWidth;
        let h = animationDiv.offsetHeight;
        renderer.setSize(w, h);
        animationDiv.appendChild(renderer.domElement);

        const scene = new THREE.Scene();
        scene.background = new THREE.Color("rgb(30,30,30)")
        const axesHelper = new THREE.AxesHelper(1);
        scene.add(axesHelper);
        const camera = new THREE.PerspectiveCamera(300, w / h, .1, 1e15);
        camera.position.set(0, 0, 1e7);
        //camera.rotation.set(0,0,Math.PI);

        const controls = new TrackballControls(camera, renderer.domElement);
        controls.rotateSpeed = 10.0
        //camera.up.set(0, 1, 0);
        camera.up = new THREE.Vector3(0, -1, 0)
        //camera.rotation.set(0, 0, Math.PI)

        const light = new THREE.AmbientLight('white', 0.05); // soft white light
        scene.add(light);

        if (sys.base.type == "earth") {
            const rEarth = 6378.1370e3;
            //const i_earth = new THREE.TextureLoader().load("./images/earth.jpeg");
            const i_earth = new THREE.TextureLoader().load("./images/earth_16k.jpg");
            const b_earth = new THREE.TextureLoader().load("./images/earth_bump_16k.jpg");
            const s_earth = new THREE.TextureLoader().load("./images/8081_earthspec4k.jpg");
            const g_earth = new THREE.SphereGeometry(rEarth, 64, 64)
            //const m_earth = new THREE.MeshPhongMaterial({ map: i_earth});
            const m_earth = new THREE.MeshPhongMaterial({ map: i_earth, bumpMap: b_earth, bumpScale: 5, specularMap: s_earth, shininess: 100 });
            const earth = new THREE.Mesh(g_earth, m_earth);
            earth.castShadow = true;
            scene.add(earth);

            const rSun = 1 * 696340000;
            const dSun = 151.39099 * 1e6 * 1e3; // 151 million km
            const g_sun = new THREE.SphereGeometry(rSun, 16, 16)
            const m_sun = new THREE.MeshBasicMaterial({ color: 'white' })
            const sun = new THREE.Mesh(g_sun, m_sun);
            sun.position.set(dSun, 0, 0);
            scene.add(sun)

            const sunlight = new THREE.PointLight('white', 1, 0, 0);
            sunlight.position.set(dSun, 0, 0);
            sunlight.castShadow = true;
            //sunlight.shadow.mapSize.width = 512; // default
            //sunlight.shadow.mapSize.height = 512; // default
            sunlight.shadow.camera.near = 1e6; // default
            sunlight.shadow.camera.far = 1e14; // default
            scene.add(sunlight);
            
            scene.background = new THREE.Color("rgb(0,0,0)")
            /* stars off for now
            const rStars = 1e14;
            const i_stars = new THREE.TextureLoader().load("./images/starfield.jpg");
            const g_stars = new THREE.SphereGeometry(rStars, 16, 16)
            const m_stars = new THREE.MeshBasicMaterial({ map: i_stars, side: THREE.BackSide, color: new THREE.Color(0.1, 0.1, 0.1) });
            const stars = new THREE.Mesh(g_stars, m_stars);
            scene.add(stars);
            */
        }

        //create time
        const sim_time_index = animData.colindex.names.indexOf("t");
        const sim_time = animData.columns[sim_time_index];

        //create bodys
        const body_keys = Object.keys(sys.bodies);

        for (let i = 0; i < body_keys.length; i++) {
            const body = sys.bodies[body_keys[i]];
            let geometry;
            if (body.geometry === 'box') {
                geometry = new THREE.BoxGeometry(
                    body.xlength,
                    body.ylength,
                    body.zlength
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
                geometry.rotateX(Math.PI / 2) //force cylinders to be z in the height direction
            }

            let material;
            if (body.material === "phong") {
                material = new THREE.MeshPhongMaterial({
                    color: body.color,
                });
            } else {
                material = new THREE.MeshBasicMaterial({
                    color: body.color,
                });
            }
            const mesh = new THREE.Mesh(geometry, material);

            mesh.receiveShadow = true;

            mesh.name = body.name;

            let q1_index = animData.colindex.names.indexOf(`${body.name}_q_base[1]`);
            let q2_index = animData.colindex.names.indexOf(`${body.name}_q_base[2]`);
            let q3_index = animData.colindex.names.indexOf(`${body.name}_q_base[3]`);
            let q4_index = animData.colindex.names.indexOf(`${body.name}_q_base[4]`);
            let r1_index = animData.colindex.names.indexOf(`${body.name}_r_base[1]`);
            let r2_index = animData.colindex.names.indexOf(`${body.name}_r_base[2]`);
            let r3_index = animData.colindex.names.indexOf(`${body.name}_r_base[3]`);

            const r1 = animData.columns[r1_index];
            const r2 = animData.columns[r2_index];
            const r3 = animData.columns[r3_index];

            let r_vector = [];
            for (let j = 0; j < sim_time.length; j++) {
                r_vector.push(r1[j]);
                r_vector.push(r2[j]);
                r_vector.push(r3[j]);
            }

            const r_interpolant = new THREE.CubicInterpolant(sim_time, r_vector, 3, [])

            let this_data = {
                q1: animData.columns[q1_index],
                q2: animData.columns[q2_index],
                q3: animData.columns[q3_index],
                q4: animData.columns[q4_index],
                r1: animData.columns[r1_index],
                r2: animData.columns[r2_index],
                r3: animData.columns[r3_index],
                r_interpolant: r_interpolant
            }

            mesh.userData = this_data;
            scene.add(mesh);


            //add body to targets menu
            $("#animationTarget").append($('<option>', {
                value: body.name,
                text: body.name,
            }))
        }

        // global for camera target, set event for when it changes
        let TARGET = scene.getObjectByName($("#animationTarget").val());

        $("#animationTarget").on("change", function () {
            TARGET = scene.getObjectByName($("#animationTarget").val());
        });

        controls.update()

        //create thrusters
        const actuator_keys = Object.keys(sys.actuators);
        for (let i = 0; i < actuator_keys.length; i++) {
            const actuator = sys.actuators[actuator_keys[i]];
            if (actuator.type == 'thruster') {

                let geometry = new THREE.ConeGeometry(0.25, 0.5, 8, 1);
                geometry.rotateX(Math.PI / 2); // force cones to be Z in the height direction                        
                geometry.translate(0, 0, -0.25);
                const material = new THREE.MeshBasicMaterial({ color: 'grey', transparent: true, opacity: 0.3 });
                const mesh = new THREE.Mesh(geometry, material);
                mesh.name = actuator.name;

                let q1_index = animData.colindex.names.indexOf(`${actuator.name}_q_base[1]`);
                let q2_index = animData.colindex.names.indexOf(`${actuator.name}_q_base[2]`);
                let q3_index = animData.colindex.names.indexOf(`${actuator.name}_q_base[3]`);
                let q4_index = animData.colindex.names.indexOf(`${actuator.name}_q_base[4]`);
                let r1_index = animData.colindex.names.indexOf(`${actuator.name}_r_base[1]`);
                let r2_index = animData.colindex.names.indexOf(`${actuator.name}_r_base[2]`);
                let r3_index = animData.colindex.names.indexOf(`${actuator.name}_r_base[3]`);
                let f_index = animData.colindex.names.indexOf(`${actuator.name}_force`);

                let this_data = {
                    q1: animData.columns[q1_index],
                    q2: animData.columns[q2_index],
                    q3: animData.columns[q3_index],
                    q4: animData.columns[q4_index],
                    r1: animData.columns[r1_index],
                    r2: animData.columns[r2_index],
                    r3: animData.columns[r3_index],
                    f: animData.columns[f_index]
                }
                mesh.userData = this_data;
                scene.add(mesh);
            }
        }
        const clock = new THREE.Clock({ autoStart: false });

        let deltaCameraPosition = new THREE.Vector3(0, 0, 10);
        controls.addEventListener("change", function () {
            deltaCameraPosition.subVectors(camera.position, TARGET.position);
        });

        const time_index = animData.colindex.names.indexOf("t");
        const time_data = animData.columns[time_index];

        let sim_elapsed_time = 0;
        let t0 = time_data[0];
        let t = t0;
        const findTimeIndex = (data) => (data > t);

        function animate() {

            ANIMATION_ID = requestAnimationFrame(animate);

            if (clock.running) {
                const sim_delta_time = PLAYBACK_SPEED * clock.getDelta();
                sim_elapsed_time += sim_delta_time;
                t = t0 + sim_elapsed_time;
                if (t > time_data[time_data.length - 1]) {
                    t = t0;
                    sim_elapsed_time = 0;
                }
            } else {
                clock.start()
            }

            const i = time_data.findIndex(findTimeIndex) - 1;

            // interp scale factor

            let j = i + 1
            while (time_data[j] - time_data[i] == 0) { j++ } //protection from when consecutive time points are identical

            const alpha = (sim_elapsed_time - time_data[i]) / (time_data[j] - time_data[i]);

            for (let b = 0; b < body_keys.length; b++) {
                let this_body = sys.bodies[body_keys[b]];
                let body = scene.getObjectByName(this_body.name);

                const current_position = new THREE.Vector3(body.userData.r1[i], body.userData.r2[i], body.userData.r3[i]);
                const next_position = new THREE.Vector3(body.userData.r1[j], body.userData.r2[j], body.userData.r3[j]);
                const interp_position = current_position.lerp(next_position, alpha)

                //const interp_position = body.userData.r_interpolant.evaluate(t);                
                body.position.set(interp_position.x, interp_position.y, interp_position.z);
                //body.position.set(interp_position[0],interp_position[1],interp_position[2]);                
                const current_quaternion = new THREE.Quaternion(body.userData.q1[i], body.userData.q2[i], body.userData.q3[i], body.userData.q4[i]);
                const next_quaternion = new THREE.Quaternion(body.userData.q1[j], body.userData.q2[j], body.userData.q3[j], body.userData.q4[j]);
                const interp_quaternion = current_quaternion.slerp(next_quaternion, alpha);
                body.quaternion.set(interp_quaternion.x, interp_quaternion.y, interp_quaternion.z, interp_quaternion.w);
            }

            for (let a = 0; a < actuator_keys.length; a++) {
                let this_actuator = sys.actuators[actuator_keys[a]];
                if (this_actuator.type == "thruster") {
                    let actuator = scene.getObjectByName(this_actuator.name);
                    if (actuator.userData.f[i] > 0) {
                        actuator.position.set(actuator.userData.r1[i], actuator.userData.r2[i], actuator.userData.r3[i]);
                        actuator.quaternion.set(actuator.userData.q1[i], actuator.userData.q2[i], actuator.userData.q3[i], actuator.userData.q4[i]);
                        actuator.visible = true;
                    } else {
                        actuator.visible = false;
                    }
                }
            }

            controls.target.copy(TARGET.position)
            camera.position.set(TARGET.position.x + deltaCameraPosition.x, TARGET.position.y + deltaCameraPosition.y, TARGET.position.z + deltaCameraPosition.z);

            controls.update();
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