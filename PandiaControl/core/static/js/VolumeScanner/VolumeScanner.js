
import { getChart } from './Chartutils.js'
import * as utils from '../utils.js'

const n_maxChartPoints = 20;
let UpdateWorkerRunning = false;
let chartVolume;
let UpdateWorker;
let ConveyorBatchIsRunning = false;
let ConveyorBatchStartTime;


//######## load data from slave ########
// is executed on page load
//@ts-ignore
if (J_SideBarMode == 'active') {
    chartVolume = getChart();
    fetch('http://' + SlaveAdress + '/getVolumeData')
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (data) {
            if (data['firstEntry'] == '') {
                utils.toggleMessage('id_EmptyLogMessage')
                //@ts-ignore
                document.getElementById("id_LogDownload").classList.add("disabled")
                //@ts-ignore
                document.getElementById("id_LogDelete").classList.add("disabled")
            }
            else {
                utils.toggleMessage()
                processVolumeDataJSON(data, 'firstEntry')
                processVolumeDataJSON(data, 'lastEntry')
                //@ts-ignore
                document.getElementById("id_LogDownload").classList.remove("disabled")
                //@ts-ignore
                document.getElementById("id_LogDelete").classList.remove("disabled")
            }
            let timestamps = data['timestamps']
            let voldata = data['volumeData']
            for (let i = 0; i < timestamps.length; i++) {
                updateGraph(timestamps[i], voldata[i]);
            }
        })


    //######### start and stop update thread ###########
    if (!UpdateWorkerRunning) {
        UpdateWorker = new Worker('/static/js/VolumeScanner/VolumeUpdateWorker.js', { type: 'module' });
        UpdateWorker.addEventListener('message', receiveUpdateMessage)
        let workerData = { "SlaveAdress": SlaveAdress, "ConveyorBatchIsRunning": ConveyorBatchIsRunning };
        UpdateWorker.postMessage(workerData)
        UpdateWorkerRunning = true
    } else {
        console.log("Update worker already running")
    }

    //############# color the button ##########
    fetch(`http://${SlaveAdress}/ThreadisRunning`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json['Message'] == 'OK') {
                //@ts-ignore
                document.getElementById("id_StartButton").style.backgroundColor = "#009933"
            }
            else {
                //@ts-ignore
                document.getElementById("id_StopButton").style.backgroundColor = "#cf2e2e"
            }
        })

    handleConveyorBelt();
}

async function VolumeLogModalButtonFunc()
{
    fetch('http://' + SlaveAdress + '/getVolumeData')
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (data) {
        if (data['firstEntry'] == '') {
            utils.toggleMessage('id_EmptyLogMessage')
            //@ts-ignore
            document.getElementById("id_LogDownload").classList.add("disabled")
            //@ts-ignore
            document.getElementById("id_LogDelete").classList.add("disabled")
        }
        else {
            utils.toggleMessage()
            processVolumeDataJSON(data, 'firstEntry')
            processVolumeDataJSON(data, 'lastEntry')
            //@ts-ignore
            document.getElementById("id_LogDownload").classList.remove("disabled")
            //@ts-ignore
            document.getElementById("id_LogDelete").classList.remove("disabled")
        }
    })
}
//@ts-ignore
window.VolumeLogModalButtonFunc = VolumeLogModalButtonFunc;

async function handleConveyorBelt(params) {
    await fetch(`http://${SlaveAdress}/ConveyorBatchIsRunning`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json['Message'] == 'OK') {
                ConveyorBatchIsRunning = true
                document.getElementById('id_ConverBeltStart')?.classList.add('active');
            }
            else {
                ConveyorBatchIsRunning = false
                document.getElementById('id_ConverBeltStart')?.classList.remove('active');
            }
        })

    if (ConveyorBatchIsRunning) {
        displayCurrentConveyorBeltStats()
    }

    let workerData = { "SlaveAdress": SlaveAdress, "ConveyorBatchIsRunning": ConveyorBatchIsRunning };
    UpdateWorker.postMessage(workerData)
}




function processVolumeDataJSON(data, key) {
    let SplitData = data[key].split(" , ")
    let timestamp = SplitData[0]
    let volumeValue = SplitData[1]
    
    let DateAndTime = timestamp.split(" ")
    let Date = DateAndTime[0]
    let Time = DateAndTime[1]
    switch (key) {
        case "firstEntry":
            //@ts-ignore
            document.getElementById("id_minDate").value = Date
            //@ts-ignore
            document.getElementById("id_minTime").value = Time.split('.')[0]
            break;
        case "lastEntry":
            //@ts-ignore
            document.getElementById("id_maxDate").value = Date
            //@ts-ignore
            document.getElementById("id_maxTime").value = Time.split('.')[0]
            break;
    }
}





//##############################################
// @ts-ignore
window.toggleOverlay = function toggleOverlay() {
    fetch('http://' + SlaveAdress + '/toggleOverlay')
        .then(utils.HandleError)
}


//#####################################################################################################################
// @todo maybe refactoring - use built-in Date object
// let time24 = new Date(newTimeStamp);
// let time12 = time24.toLocaleTimeString()
function updateGraph(newTimeStamp, newDataPoint) {
    let time24 = newTimeStamp.split(' ')[1].split('.')[0]; //"2022-01-26 14:42:16.030294" to "14:42:16"
    // let hour = time24.substring(0, 2) * 1
    // let time12;
    // if (hour >= 13) {
    //     hour -= 12
    //     time12 = hour.toString() + time24.substring(2) + " PM"
    // }
    // else
    //     time12 = time24 + " AM"
    // chartVolume.data.labels.push("");
    chartVolume.data.labels.push(time24);
    var rounded = Math.round(newDataPoint * 100) / 100
    chartVolume.data.datasets[0].data.push(rounded);
    if (chartVolume.data.datasets[0].data.length > n_maxChartPoints) {
        chartVolume.data.datasets[0].data.splice(0, 1);
        chartVolume.data.labels.splice(0, 1);
    }
    chartVolume.update();
    // @ts-ignore
    id_CurrentVolume.innerHTML = "Current volume in liters: " + rounded
}



export function displayCurrentConveyorBeltStats() {
    //display conveyor belt speed on active screen
    fetch(`http://${SlaveAdress}/getVolumeSettings`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            //@ts-ignore
            document.getElementById("id_BeltSpeedVis").innerHTML = "Belt Speed: " + json['BeltSpeed'] + " m/s";
        });

    fetch(`http://${SlaveAdress}/getConveyorBatchStartTime`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            //convert from e.g. 2022-07-12 13:46:13.600489 to 12-07-2022 13:46:13
            let dateTime = json['ConveyorBatchStartTime'].split(" ")
            let date = dateTime[0].split("-")
            let time = dateTime[1].split(".")
            let out = date[2] + "-" + date[1] + "-" + date[0] + " " + time[0]
            //@ts-ignore
            document.getElementById("id_BeltStartTime").innerHTML = "Start Time: " + out;
            ConveyorBatchStartTime = json['ConveyorBatchStartTime'];
        });
}


async function startConveyorBatch() {
    document.getElementById('id_ConverBeltStart')?.classList.add('active')
    document.getElementById('id_ConverBeltStop')?.classList.remove('active')
    document.getElementById('id_ConverBeltReset')?.classList.remove('active')
    await fetch(`http://${SlaveAdress}/startConveyorBatch`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
        });

    handleConveyorBelt();

}
//@ts-ignore
window.startConveyorBatch = startConveyorBatch;

async function stopConveyorBatch() {
    document.getElementById('id_ConverBeltStart')?.classList.remove('active')
    document.getElementById('id_ConverBeltStop')?.classList.add('active')
    document.getElementById('id_ConverBeltReset')?.classList.remove('active')
    await fetch(`http://${SlaveAdress}/stopConveyorBatch`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
        });
    handleConveyorBelt();
}
//@ts-ignore
window.stopConveyorBatch = stopConveyorBatch;



async function resetConveyorBatch() {
    document.getElementById('id_ConverBeltStart')?.classList.remove('active')
    document.getElementById('id_ConverBeltStop')?.classList.remove('active')
    document.getElementById('id_ConverBeltReset')?.classList.add('active')
    await fetch(`http://${SlaveAdress}/resetConveyorBatch`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
        });
    handleConveyorBelt();

    document.getElementById("id_BeltStartTime").innerHTML = "Start Time: ";
    document.getElementById("id_BeltTime").innerHTML = "Time in Batch: ";
    document.getElementById("id_BeltSpeedVis").innerHTML = "Belt Speed: ";
    document.getElementById("id_BeltTotalVolume").innerHTML = "Total Volume Flow: ";
        

}
//@ts-ignore
window.resetConveyorBatch = resetConveyorBatch;


function updateConveyorBeltBatch(newTotalVolumeFlow) {
    let timestamp = Date.parse(newTotalVolumeFlow["timestamp"])
    let totalVolumeFlow = newTotalVolumeFlow["newTotalVolumeFlow"]
    let t_p = (timestamp - Date.parse(ConveyorBatchStartTime)) / 1000
    // console.log(totalVolumeFlow);

    // @ts-ignore
    id_BeltTotalVolume.innerHTML = "Total Volume Flow: " + Math.round(100*totalVolumeFlow)/100 + " l";
    // @ts-ignore
    id_BeltTime.innerHTML = "Time in Batch: " + t_p + " s";
    // @ts-ignore
    id_BeltSpeedVis.innerHTML = "Belt Speed: " + newTotalVolumeFlow['BeltSpeed'] + " m/s";
}

function getDataFromEvent(e) {
    return e.data.data
}

function receiveUpdateMessage(e) {
    switch (e.data.type) {
        case "newVolume":
            let data = JSON.parse(getDataFromEvent(e))
            if (typeof data.timestamp !== 'undefined') {
                updateGraph(data.timestamp, data.volume);
            }
            break;
        case "newImage":
            //@ts-ignore
            if (J_DisplayMode == "image") {
                let img_data = JSON.parse(getDataFromEvent(e))
                if (img_data.image.length != 0) {
                    //@ts-ignore
                    id_ActiveImage.src = "data:image/jpg;charset=utf-8;base64," + img_data.image;
                }
            }
            break;

        case "newTotalVolumeFlow":
            let data2 = JSON.parse(getDataFromEvent(e))
            updateConveyorBeltBatch(data2);
            break;
        default:
            break;
    }

}

function startVolumeMeasurement() {
    //start a python thread
    fetch('http://' + SlaveAdress + '/startVolumeThread')
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json['Message'] == 'OK') {
                document.getElementById("id_StartButton").style.backgroundColor = "#009933"
                document.getElementById("id_StopButton").style.backgroundColor = "black"
            }
            else {
                window.alert("Please select a 3D Silo model before starting the measurement.")
                return;
            }
        })
}
//@ts-ignore
window.startVolumeMeasurement = startVolumeMeasurement;

function stopVolumeMeasurement() {
    fetch('http://' + SlaveAdress + '/stopVolumeThread')
        .then(utils.HandleError)
        .then(function () {
            document.getElementById("id_StartButton").style.backgroundColor = "black"
            document.getElementById("id_StopButton").style.backgroundColor = "#cf2e2e"
        })
}
//@ts-ignore
window.stopVolumeMeasurement = stopVolumeMeasurement;

function getVolumeLog() {
    utils.toggleMessage('id_VolumeLogProgress')
    //@ts-ignore
    let minDateTime = document.getElementById('id_minDate').value + " " + document.getElementById('id_minTime').value
    //@ts-ignore
    let maxDateTime = document.getElementById('id_maxDate').value + " " + document.getElementById('id_maxTime').value
    const data = { 'minDateTime': minDateTime, 'maxDateTime': maxDateTime }
    fetch(`http://${SlaveAdress}/getVolumeLog`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data)
    })
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            let a = document.createElement('a');
            let data = new Blob(json)
            a.href = window.URL.createObjectURL(data);
            a.download = 'VolumeData.csv';
            // a.style.display = 'none';
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            utils.toggleMessage()
        })
        .catch(function (error) {
            console.log(error);
            utils.toggleMessage('id_LogErrorMessage')
        })
}
//@ts-ignore
window.getVolumeLog = getVolumeLog;

function deleteVolumeLog()
{
    if (confirm("Are you sure that you want to delete ALL collected Volume Data?")) {
        utils.toggleMessage('id_VolumeLogProgress')
        fetch(`http://${SlaveAdress}/deleteVolumeLog`)
        .then(utils.HandleError)
        .then(function () {
            //@ts-ignore
            document.getElementById('id_minDate').value = ""
            //@ts-ignore
            document.getElementById('id_minTime').value = ""
            //@ts-ignore
            document.getElementById('id_maxDate').value = ""
            //@ts-ignore
            document.getElementById('id_maxTime').value = ""
            utils.toggleMessage()
            location.reload();
        })
        .catch(function (error) {
            console.log(error);
            utils.toggleMessage('id_LogErrorMessage')
        })
    }
}
//@ts-ignore
window.deleteVolumeLog = deleteVolumeLog;