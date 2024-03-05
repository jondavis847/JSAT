import cytoscape from 'https://cdn.jsdelivr.net/npm/cytoscape@3.27.0/+esm';
import edgehandles from 'https://cdn.jsdelivr.net/npm/cytoscape-edgehandles@4.0.1/+esm';
cytoscape.use(edgehandles);

function cy_autosize(node) {
    let label_length = node.data('label').length * 7;
    let final_length = (label_length > 50) ? label_length : 50;
    return final_length
}

export var cy = cytoscape({
    container: $("#cyCanvasDiv"),
    style: [
        {
            selector: '*',
            style: {
                'font-size': 12,
                'text-valign': 'center',
                'text-halign': 'center',
                'label': 'data(label)',
                'border-width': 3
            }
        },
        {
            selector: '.base',
            style: {
                'shape': 'round-rectangle',
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
                'width': cy_autosize,
                'height': 50,
            }
        },
        {
            selector: '.joint',
            style: {
                'shape': 'ellipse',
                'background-color': 'aqua',
                'width': cy_autosize,
                'height': 50,
            }
        },
        {
            selector: 'edge',
            style: {
                'width': 3,
                'line-color': '#ccc',
                'target-arrow-color': 'whitesmoke',
                'target-arrow-shape': 'triangle',
                'curve-style': 'bezier',
            }
        },
        {
            selector: '.actuator',
            style: {
                'shape': 'round-rectangle',
                'background-color': '#FDFD96',
                'width': cy_autosize,
                'height': 50,
            }
        },
        {
            selector: '.sensor',
            style: {
                'shape': 'round-rectangle',
                'background-color': '#d600ff',
                'width': cy_autosize,
                'height': 50,
            }
        },
        {
            selector: '.gravity',
            style: {
                'shape': 'round-rectangle',
                'background-color': '#966FD6',
                'width': cy_autosize,
                'height': 50,
            }
        },
        {
            selector: '.software',
            style: {
                'shape': 'round-rectangle',
                'background-color': '#FFD1DC',
                'width': cy_autosize,
                'height': 50,
            }
        },
        {
            selector: '.model',
            style: {
                'shape': 'round-rectangle',
                'background-color': '#001eff',
                'width': cy_autosize,
                'height': 50,
            }
        },
        { // just to suppress a warning with edgehandles https://github.com/cytoscape/cytoscape.js-edgehandles/issues/119
            //didnt work
            selector: '.eh-ghost-node',
            style: {
                'label': ''
            }
        },
        { // just to suppress a warning with edgehandles https://github.com/cytoscape/cytoscape.js-edgehandles/issues/119
            //didnt work
            selector: '.eh-handle-node',
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
export let eh = cy.edgehandles({ snap: false });
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

export function clearCanvas() {
    cy.elements().remove();
    JSAT.base = {};
    JSAT.bodies = {};
    JSAT.joints = {};
    JSAT.actuators = {};
    JSAT.software = {};
    JSAT.sensors = {};
    JSAT.gravity = {};
};



cy.on('select', (evt) => {
    evt.target.style({ 'border-color': 'yellow' });
});

cy.on('unselect', (evt) => {
    evt.target.style({ 'border-color': 'black' });
});


/*
cy.on('tap', 'node', function (evt) {
    evt.target.select();
    console.log(cy.$('node').selected())
});
*/