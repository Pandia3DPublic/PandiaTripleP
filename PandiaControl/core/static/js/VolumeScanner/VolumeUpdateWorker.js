let LocalSlaveAdress
let ConveyorBatchIsRunning

function WorkerFunction() {
    fetch('http://' + LocalSlaveAdress + '/getNewVolume')
        .then(HandleError)
        .then(resp => resp.json())
        .then(function (json) {
            self.postMessage({ type: "newVolume", data: JSON.stringify(json) })
        })

    fetch('http://' + LocalSlaveAdress + '/getNewVolumeImage')
        .then(HandleError)
        .then(resp => resp.json())
        .then(function (json) {
            self.postMessage({ type: "newImage", data: JSON.stringify(json) })
        })

    if (ConveyorBatchIsRunning) {
        fetch('http://' + LocalSlaveAdress + '/getNewTotalVolumeFlowofCurrentConveyorBatch')
            .then(HandleError)
            .then(resp => resp.json())
            .then(function (json) {
                self.postMessage({ type: "newTotalVolumeFlow", data: JSON.stringify(json) })
            })
    }
    setTimeout(WorkerFunction, 1000);
}


onmessage = function (e) {
    LocalSlaveAdress = e.data["SlaveAdress"]
    ConveyorBatchIsRunning = e.data["ConveyorBatchIsRunning"]
    WorkerFunction();
}


// IMPORTANT: Error handling functions were copied here from utils.js (without window.alert functions)
// check for all response errors
function HandleError(response) {
    if (!response.ok) {
        //Python error
        if (response.status == 420) {
            throw new Error(`Error occured (status ${response.status})`)
        }
        //Misc error
        throw new Error(`Error occured in ${response.url} (status ${response.status})`);
    }
    return response;
}
