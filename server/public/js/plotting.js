import "https://cdn.jsdelivr.net/npm/plotly.js/dist/plotly.min.js";

export function plotStateData() {
    if ($("#plotState").data("simOrFile") == 0) {
        plotSimData();
    } else if ($("#plotState").data("simOrFile") == 1) {
        plotFileData();
    }
}

export function plotSimData() {
    const colormap = getColorMap();

    let selectedSims = []
    $("#simSelect").find(":selected").each(function () { selectedSims.push($(this).val()) })
    let selectedXState = $("#xStateSelect").find(":selected").val();
    let selectedYStates = []
    $("#yStateSelect").find(":selected").each(function () { selectedYStates.push($(this).val()) })
    const selectedStates = [...new Set([selectedXState, ...selectedYStates])]
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

        Plotly.newPlot("plotsDiv", traces, getLayout(selectedXState, selectedYStates), { scrollZoom: true })
    });

    xhr.open("POST", "/plotstates");
    xhr.send(JSON.stringify({ sims: selectedSims, states: selectedStates }));
}

export function plotFileData() {
    const colormap = getColorMap();
    let selectedXState = $("#xStateSelect").find(":selected");
    //let selectedYStates = [];
    let selectedYStates = $("#yStateSelect option:selected")//.find(":selected") //.each(function () { selectedYStates.push($(this)) });    
    let traces = [];
    let colorCtr = 0;
    selectedYStates.each(function () {

        //check if ydata is valid
        const xdata = selectedXState.data("values");
        const ydata = $(this).data("values");
        let final_x = [];
        let final_y = [];
        for (let i = 0; i < xdata.length; i++) {
            if (ydata[i] != null) {
                final_x.push(xdata[i]);
                final_y.push(ydata[i]);
            }
        }


        traces.push({
            x: final_x,
            y: final_y,
            type: 'scatter',
            mode: 'lines',
            name: $(this).text(),
            showlegend: true,
            line: {
                color: `rgb(${colormap[colorCtr]})`,
                width: 1
            },
        })

        colorCtr++;
        if (colorCtr >= colormap.length) {
            colorCtr = 0
        }
    })
    let ystates = []
    selectedYStates.each(function () { ystates.push($(this).text()) })
    Plotly.newPlot("plotsDiv", traces, getLayout(selectedXState, ystates), { scrollZoom: true })
}

function getLayout(xstate, ystate) {
    let layout = {
        plot_bgcolor: "rgb(35, 36, 36)",
        paper_bgcolor: "rgb(35, 36, 36)",
        font: {
            family: 'Courier New, monospace',
            size: 10,
            color: 'aliceblue'
        },
        showlegend: true,
        legend: {
            orientation: "h",
            bgcolor: 'rgba(0,0,0,0)' //transparent
        },
        xaxis: {
            title: xstate,
            gridcolor: "rgb(0,0,0)"
        },
        yaxis: {
            title: {
                text: ystate.join('<br>'),
                automargin: true
            },
            gridcolor: "rgb(0,0,0)"
        },
        margin: {
            //l: 50,
            r: 50,
            b: 50,
            t: 50,
            pad: 4
        },
    }
    return layout
}

function getColorMap() {
    const colormap = [
        [0, 255, 159],
        [0, 184, 255],
        [189, 0, 255],
        [255, 95, 31],
        [0, 30, 255],
        [234, 0, 217],
    ];
    return colormap
}