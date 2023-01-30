import * as utils from '../utils.js'
import * as THREE from '../../3rdpartylibs/three.js-master/build/three.module.js'
import * as Renderer from '../Renderer.js'

// main functions

export function disableShowButtons()
{
    try { document.getElementById("id_ShowScanAndCAD").classList.add('disabled') } catch { };
    try { document.getElementById("id_ShowExtra").classList.add('disabled') } catch { };
    try { document.getElementById("id_ShowMissing").classList.add('disabled') } catch { };
    try { document.getElementById("id_ShowAllDifferences").classList.add('disabled') } catch { };
    try { document.getElementById("id_ShowVisibility").classList.add('disabled') } catch { };
}
export function enableShowButtons()
{
    try { document.getElementById("id_ShowScanAndCAD").classList.remove('disabled') } catch { };
    try { document.getElementById("id_ShowExtra").classList.remove('disabled') } catch { };
    try { document.getElementById("id_ShowMissing").classList.remove('disabled') } catch { };
    try { document.getElementById("id_ShowAllDifferences").classList.remove('disabled') } catch { };
    try { document.getElementById("id_ShowVisibility").classList.remove('disabled') } catch { };
}


let GoodFitnessThreshold = 0.70;

function handleAlignmentInfocard(fitness = -1) {
    let alignStatus = document.getElementById("id_InfoCard_Alignment")
    let infoCard = document.getElementById("id_InfoCard");
    infoCard.classList.remove("success")
    infoCard.classList.remove("fail")
    console.log("fitness: " + fitness)
    if (fitness == -1) //no fitness provided, for reset
    {
        alignStatus.innerText = "Pending"
    }
    else if (fitness > GoodFitnessThreshold)
    {
        // alignStatus.innerText = "OK"
        alignStatus.innerText = (fitness * 100).toFixed(2) + "%"
        infoCard.classList.add("success")
    }
    else {
        // alignStatus.innerText = "Failed"
        alignStatus.innerText = (fitness * 100).toFixed(2) + "%"
        infoCard.classList.add("fail")
    }
}

function handleAlignmentButtonsAndModel(fitness) {
    if (fitness > GoodFitnessThreshold)
    {
        enableShowButtons();
        //@ts-ignore
        if (document.getElementById('id_AllDifferencesCheckbox').checked === true)
            $('#id_ShowAllDifferences').trigger('click');
        else
            $('#id_ShowScanAndCAD').trigger('click');
    }
    else {
        disableShowButtons();
        Renderer.removeAllModels()
        Renderer.addPCDsToScene();
    }
}

export function setModelNameInInfocard(modelname)
{
    if (modelname !== undefined)
    {
        try{
            document.getElementById('id_InfoCard_Model').innerText = modelname;
        }
        catch { console.log("Failed to set model name in infocard"); }
    }
}

async function StartScanButtonFunc() 
{
    disableShowButtons()
    handleAlignmentInfocard();
    try { document.getElementById("id_AddScan").classList.add('disabled') } catch { };
    try { document.getElementById("id_RevertLastScan").classList.add('disabled') } catch { };
    utils.toggleMessage("id_ScanningProgress");
    Renderer.removeAllModels();
    Renderer.Scene.add(Renderer.CurrentReferenceModel)
    Renderer.clearPCDsAndColors();
    let success = true;

    let time = new Date();
    let fitness = -1;
    success = await fetch(`http://${SlaveAdress}/startScan`)
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (json) {
        if (json.hasOwnProperty('Message')) {
            if (json['Message'] == 'Failed') {
                utils.toggleMessage("id_ScanningFail")
                return false;
            }
            if (json['Message'] == "ReferenceModelNotSet") {
                utils.toggleMessage("id_ScanningFailModel")
                return false;
            }
        }
        fitness = json['fitness'] * 1;
        return true;
    });
    utils.printElapsedTime(time, "startScan")
    if (!success) { console.log("startScan failed"); return success; }


    success = await getLatestCameraPCDs()
    if (!success) { console.log("getLatestCameraPCDs failed"); return success; }
    success = await getLatestGradientColors();
    if (!success) { console.log("getLatestGradientColors failed"); return success; }

    if (fitness <= GoodFitnessThreshold)
    {
        handleAlignmentInfocard(fitness);
        handleAlignmentButtonsAndModel(fitness);
        utils.toggleMessage();
        try { document.getElementById("id_AddScan").classList.remove('disabled') } catch { };
        return false;
    }
    
    success = await getMissingPoints()
    if (!success) { console.log("getMissingPoints failed"); return success; }

    handleAlignmentInfocard(fitness);
    handleAlignmentButtonsAndModel(fitness);

    try { document.getElementById("id_AddScan").classList.remove('disabled') } catch { };
    utils.toggleMessage();
}
//@ts-ignore
window.StartScanButtonFunc = StartScanButtonFunc;


async function AddScan() 
{
    disableShowButtons()
    try { document.getElementById("id_AddScan").classList.add('disabled') } catch { };
    try { document.getElementById("id_RevertLastScan").classList.add('disabled') } catch { };
    utils.toggleMessage("id_ScanningProgress");
    let success = true;

    let time = new Date();
    let fitness = -1;
    success = await fetch(`http://${SlaveAdress}/addScan`)
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (json) {
        if (json.hasOwnProperty('Message') && json['Message'] == 'Failed') {
            utils.toggleMessage("id_ScanningFail")
            return false;
        }
        fitness = json['fitness'] * 1;
        return true;
    });
    utils.printElapsedTime(time, "addScan")
    if (!success) { console.log("addScan failed"); return success; }

    success = await getLatestCameraPCDs()
    if (!success) { console.log("getLatestCameraPCDs failed"); return success; }
    success = await getLatestGradientColors()
    if (!success) { console.log("getLatestGradientColors failed"); return success; }

    if (fitness <= GoodFitnessThreshold)
    {
        handleAlignmentInfocard(fitness);
        handleAlignmentButtonsAndModel(fitness);
        utils.toggleMessage();
        try { document.getElementById("id_RevertLastScan").classList.remove('disabled') } catch { };
        return false;
    }

    success = await getMissingPoints()
    if (!success) { console.log("getMissingPoints failed"); return success; }

    handleAlignmentInfocard(fitness);
    handleAlignmentButtonsAndModel(fitness);

    try { document.getElementById("id_AddScan").classList.remove('disabled') } catch { };
    try { document.getElementById("id_RevertLastScan").classList.remove('disabled') } catch { };
    utils.toggleMessage();

}
//@ts-ignore
window.AddScan = AddScan;

async function RevertLastScan() 
{
    disableShowButtons()
    try { document.getElementById("id_AddScan").classList.add('disabled') } catch { };
    try { document.getElementById("id_RevertLastScan").classList.add('disabled') } catch { };
    utils.toggleMessage("id_ScanningProgress");
    let success = true;

    let time = new Date();
    let fitness = -1;
    success = await fetch(`http://${SlaveAdress}/revertLastScan`)
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (json) {
        if (json.hasOwnProperty('Message') && json['Message'] == 'Failed') {
            utils.toggleMessage("id_ScanningFail")
            return false;
        }
        fitness = json['fitness'] * 1;
        return true;
    });
    utils.printElapsedTime(time, "revertLastScan")
    if (!success) { console.log("revertLastScan failed"); return success; }



    success = await fetch(`http://${SlaveAdress}/getNumberOfCameras`)
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (ncam) {
        for (let i =0; i< ncam;i++){
            Renderer.PCDArray.pop();
            Renderer.PCDColorsArray.pop();
            Renderer.PCDGradientArray.pop();
        }
        return true;
    });

    success = await getMissingPoints()
    if (!success) { console.log("getMissingPoints failed"); return success; }

    handleAlignmentInfocard(fitness);
    handleAlignmentButtonsAndModel(fitness);

    try { document.getElementById("id_RevertLastScan").classList.add('disabled') } catch { };
    try { document.getElementById("id_AddScan").classList.remove('disabled') } catch { };
    utils.toggleMessage();
}
//@ts-ignore
window.RevertLastScan = RevertLastScan;


function ShowExtraParts() 
{
    utils.toggleMessage();
    Renderer.removeAllModels();
    Renderer.Scene.add(Renderer.CurrentReferenceModel)
    Renderer.addPCDsToSceneGradient();
}
//@ts-ignore
window.ShowExtraParts = ShowExtraParts;


function ShowMissingParts() 
{
    utils.toggleMessage();
    Renderer.removeAllModels();
    Renderer.Scene.add(Renderer.CurrentReferenceModel);
    Renderer.Scene.add(Renderer.MissingPoints);
    // Renderer.Scene.add(Renderer.CurrentPCD);
    // Renderer.CameraModel.visible = true;
    // Renderer.Scene.add(Renderer.CameraModel);
}
//@ts-ignore
window.ShowMissingParts = ShowMissingParts;


function ShowAllDifferences() 
{
    utils.toggleMessage();
    Renderer.removeAllModels();
    Renderer.Scene.add(Renderer.CurrentReferenceModel)
    Renderer.Scene.add(Renderer.MissingPoints);
    Renderer.addPCDsToSceneGradient();
}
//@ts-ignore
window.ShowAllDifferences = ShowAllDifferences;


function ShowScanAndPCD()
{
    utils.toggleMessage();
    Renderer.removeAllModels()
    Renderer.Scene.add(Renderer.CurrentReferenceModel)
    Renderer.addPCDsToScene();
}
//@ts-ignore
window.ShowScanAndPCD = ShowScanAndPCD;


function ShowVisibility()
{
    let time = new Date();
    utils.toggleMessage("id_ScanningProgress");
    Renderer.removeAllModels()
    fetch(`http://${SlaveAdress}/getVisibilityPCD`)
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (json) {
        if (json.hasOwnProperty('Message') && json['Message'] == 'Failed') {
            utils.toggleMessage("id_ScanningFailMisc")
            return false;
        }
        const buffer = new THREE.BufferGeometry();
        const PointMaterial = new THREE.PointsMaterial({ size: G_PointSize, vertexColors: true });
        const pcd = new THREE.Points(buffer, PointMaterial);

        buffer.setAttribute('position', new THREE.Float32BufferAttribute(json.points, 3));
        buffer.setAttribute('color', new THREE.Float32BufferAttribute(json.colors, 3));

        Renderer.setVisibilityPCD(pcd);
        Renderer.Scene.add(Renderer.VisibilityPCD);
        utils.printElapsedTime(time, "getVisibilityPCD")
        utils.toggleMessage()
    });
}
//@ts-ignore
window.ShowVisibility = ShowVisibility;


async function getLatestCameraPCDs()
{
    let time = new Date();
    return fetch(`http://${SlaveAdress}/getLatestCameraPCDs`)
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (json) {
        if (json.hasOwnProperty('Message') && json['Message'] == 'Failed') {
            utils.toggleMessage("id_ScanningFailMisc")
            return false;
        }
        let smallestVoxelSize = 0
        for (let jsonpcd of json) {
            if (smallestVoxelSize == 0 ||  jsonpcd['ApproxVoxelSize']< smallestVoxelSize){
                smallestVoxelSize = jsonpcd['ApproxVoxelSize']
            }
        }
        for (let jsonpcd of json) {
            const buffer = new THREE.BufferGeometry();
            let pointsize = 0
            if (document.getElementById('id_DynamicPointSizeCheckbox').checked === true){
                pointsize = 1000* jsonpcd['ApproxVoxelSize'] 
            } else{
                pointsize = G_PointSize
            }
            const PointMaterial = new THREE.PointsMaterial({ size: pointsize, vertexColors: true });
            const pcd = new THREE.Points(buffer, PointMaterial);
            
            const colorBuffer = new THREE.Float32BufferAttribute(jsonpcd.colors, 3);
            buffer.setAttribute('color', colorBuffer);
            buffer.setAttribute('position', new THREE.Float32BufferAttribute(jsonpcd.points, 3));
    
            Renderer.pushToPCDArray(pcd);
            Renderer.pushToPCDColorsArray(colorBuffer);
        }


        utils.printElapsedTime(time, "getLatestCameraPCDs")
        return true;
    });
}


async function getLatestGradientColors()
{
    let time = new Date();
    return fetch(`http://${SlaveAdress}/getLatestGradientColors`)
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (json) {
        if (json.hasOwnProperty('Message') && json['Message'] == 'Failed') {
            utils.toggleMessage("id_ScanningFailMisc")
            return false;
        }

        for (let jsoncolors of json) {
            Renderer.pushToPCDGradientArray(new THREE.Float32BufferAttribute(jsoncolors.colors, 3))
        }

        if (Renderer.PCDGradientArray.length != Renderer.PCDArray.length)
        {
            console.log("Warning: PCD and gradient array sizes differ!")
        }
        utils.printElapsedTime(time, "getLatestGradientColors")
        return true;
    });
}

async function getMissingPoints()
{
    let time = new Date();
    return fetch(`http://${SlaveAdress}/getMissingPoints`)
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (json) {
        if (json.hasOwnProperty('Message') && json['Message'] == 'Failed') {
            utils.toggleMessage("id_ScanningFailMisc")
            return false;
        }
        const buffer = new THREE.BufferGeometry();
        const PointMaterial = new THREE.PointsMaterial({ size: G_PointSize, color: new THREE.Color(0, 0, 1) });
        const pcd = new THREE.Points(buffer, PointMaterial);

        buffer.setAttribute('position', new THREE.Float32BufferAttribute(json.points, 3));

        Renderer.setMissingPoints(pcd);
        utils.printElapsedTime(time, "getMissingPoints")
        return true;
    })
}