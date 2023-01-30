import * as Renderer from '../Renderer.js'
import * as utils from '../utils.js'
import * as THREE from '../../3rdpartylibs/three.js-master/build/three.module.js'
import { setupFOVSliders, FOVSliderFunc, getAndShowFOVPcd, recordAndShowFOVPcd, getFOVImage } from '../Sidebar.js'


async function FOVCameraSelectFunction() {
    await FOVsetCurrentCamera();
    await setupFOVSliders();
    FOVSliderFunc()
}
//@ts-ignore
window.FOVCameraSelectFunction = FOVCameraSelectFunction;

async function ThresholdCameraSelectFunction() {
    await ThresholdsetCurrentCamera();
    changeToleranceBoxValues(); 
    // FOVSliderFunc()
}
//@ts-ignore
window.ThresholdCameraSelectFunction = ThresholdCameraSelectFunction;



function changeToleranceBoxValues()
{
    return fetch(`http://${SlaveAdress}/getCameraSettings`, 
    {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({'serialnumber': G_CurrentCamera})
    })
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (json) {
        //@ts-ignore
        document.getElementById("id_ColorThresholdMin").value = json['ColorThresholdMin'];
        //@ts-ignore
        document.getElementById("id_ColorThresholdMax").value = json['ColorThresholdMax'];
        //@ts-ignore
        utils.updateValueLabels(json['ColorThresholdMin'],'id_ColorThresholdMin');
        utils.updateValueLabels(json['ColorThresholdMax'],'id_ColorThresholdMax');
        return true;
    })
}



async function FOVButtonFunc() {
    utils.toggleMessage('id_FOVProgress')
    Renderer.removeAllModels();
    Renderer.Scene.add(Renderer.CameraModel)

    let serials = await fetch('/getCamsOfCurrentSystem')
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            let serials = json['serialnumbers']
            $("#id_FOVCameraSelect").empty()
            let CameraSelect = document.getElementById('id_FOVCameraSelect')
            serials.forEach(serialnumber => {
                let newOption = document.createElement('option')
                newOption.text = serialnumber
                CameraSelect.appendChild(newOption)
            });
            $("#id_FOVCameraSelect").children().first().selected = true
            FOVsetCurrentCamera();
            return serials;
        })

    await setupFOVSliders();
    await getAndShowFOVPcd();
    await getFOVImage();
    utils.toggleMessage()
}
//@ts-ignore
window.FOVButtonFunc = FOVButtonFunc


async function FOVsetCurrentCamera() {
    G_CurrentCamera = $('#id_FOVCameraSelect').find(":selected").text();
    console.log(G_CurrentCamera);
}
//@ts-ignore
window.FOVsetCurrentCamera = FOVsetCurrentCamera

async function ThresholdsetCurrentCamera() {
    G_CurrentCamera = $('#id_ThresholdCameraSelect').find(":selected").text();
    console.log(G_CurrentCamera);
}
//@ts-ignore
window.ThresholdsetCurrentCamera = ThresholdsetCurrentCamera

function ReferencingButtonFunc() {
    document.getElementById("id_RenderCanvas").setAttribute('hidden', 'hidden');
    document.getElementById("id_Referencing_images-container").removeAttribute('hidden');
    utils.toggleMessage()
    Renderer.removeAllModels();

    fetch(`http://${SlaveAdress}/getQualitySettings`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            //@ts-ignore
            document.getElementById("id_Pattern_h").value = json['Pattern_h'];
            //@ts-ignore
            document.getElementById("id_Pattern_w").value = json['Pattern_w'];
            //@ts-ignore
            document.getElementById("id_SquareLength").value = json['SquareLength'];
        })
}
//@ts-ignore
window.ReferencingButtonFunc = ReferencingButtonFunc


async function StartReferencingButtonFunc() {
    $.blockUI({ message: `<h1> <img src=${G_BLOCKUI_IMG_PATH} alt=''/> Referencing...</h1>` });
    utils.toggleMessage("id_ReferencingProgress");
    utils.removeAllChildNodes("id_Referencing_images-container")
    document.getElementById("id_RenderCanvas").setAttribute('hidden', 'hidden');
    document.getElementById("id_Referencing_images-container").removeAttribute('hidden');
    document.getElementById('id_ShowCalibrationPCD').classList.add('disabled')

    let timePositions = new Date()
    let cameraPositionsJson
    let referencingSuccess = await fetch(`http://${SlaveAdress}/computeCameraPositions`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            cameraPositionsJson = json;
            utils.printElapsedTime(timePositions, "computeCameraPositions")
            if (cameraPositionsJson.hasOwnProperty('Message')) {
                return false; //referencing failed case
            }
            else{
                return true;
            }
        })
        .catch(function () {
            $.unblockUI()
        })

    let timeImages = new Date()
    //get corner images to visualize result regardless of success
    await fetch(`http://${SlaveAdress}/getCornerImages`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (imagesJson) {
            if (imagesJson.hasOwnProperty('Message') && imagesJson['Message'] == 'Failed') {
                console.log('Getting corner images failed');
                return;
            }
            //get images
            for (var key in imagesJson) {
                let image = imagesJson[key]['image']
                let serial = imagesJson[key]['serialnumber'] //todo show serialnumber as image overlay
                if (image.length != 0) {
                    var img = document.createElement('img');
                    img.setAttribute('src', "data:image/jpg;charset=utf-8;base64," + image)
                    document.getElementById('id_Referencing_images-container').appendChild(img);
                }
                else {
                    console.log("corner image is empty");
                }
            }
            //resize image elements
            let children = document.getElementById("id_Referencing_images-container").children
            if (children.length == 1) {
                children[0].setAttribute('width', '100%')
            }
            if (children.length >= 2 && children.length <= 4) {
                for (let child of children) {
                    child.setAttribute('width', '50%')
                }
            }
            else if (children.length > 4) {
                for (let child of children) {
                    child.setAttribute('width', '33.3%')
                }
            }
            utils.printElapsedTime(timeImages, "getCornerImages")
        })
        .catch(function () {
            $.unblockUI()
        })

    if (referencingSuccess) {
        //update db entries
        for (var cameraKey in cameraPositionsJson) {
            await fetch('/changeCameraSettings', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(cameraPositionsJson[cameraKey])
            })
                .then(utils.HandleError)
                .then(response => response.json())
                .then(function (json) {
                    if (json['Message'] != 'OK') {
                        utils.toggleMessage("id_ReferencingFailDB");
                    }
                })
                .catch(function () {
                    $.unblockUI()
                })
        }
        document.getElementById('id_ShowCalibrationPCD').classList.remove('disabled')
        utils.toggleMessage("id_ReferencingSuccess");
    }
    else {
        utils.toggleMessage("id_ReferencingFail")
    }
    $.unblockUI()
}
//@ts-ignore
window.StartReferencingButtonFunc = StartReferencingButtonFunc;

function ShowCalibrationPCD() {
    utils.toggleMessage('id_ReferencingProgress');
    document.getElementById("id_RenderCanvas").removeAttribute('hidden');
    document.getElementById("id_Referencing_images-container").setAttribute('hidden', 'hidden');
    Renderer.removeAllModels()

    fetch(`http://${SlaveAdress}/getCalibrationPCD`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json.hasOwnProperty('Message') && json['Message'] != 'OK') {
                console.log('getting calibration pcd failed');
                utils.toggleMessage('id_ReferencingFailMisc');
                return false;
            }

            const buffer = new THREE.BufferGeometry();
            const PointMaterial = new THREE.PointsMaterial({ size: G_PointSize, vertexColors: true });
            const pcd = new THREE.Points(buffer, PointMaterial);

            const colors = new THREE.Float32BufferAttribute(json.colors, 3);
            buffer.setAttribute('position', new THREE.Float32BufferAttribute(json.points, 3));
            buffer.setAttribute('color', new THREE.Float32BufferAttribute(json.colors, 3));

            Renderer.setCalibrationPCD(pcd);
            Renderer.Scene.add(Renderer.CalibrationPCD)
            utils.toggleMessage()
            return true;
        })
}
//@ts-ignore
window.ShowCalibrationPCD = ShowCalibrationPCD;


function setReferencingSettings() {
    //@ts-ignore
    let Pattern_h = document.getElementById("id_Pattern_h").value * 1;
    //@ts-ignore
    let Pattern_w = document.getElementById("id_Pattern_w").value * 1;
    //@ts-ignore
    let SquareLength = document.getElementById("id_SquareLength").value * 1;

    let data = {
        'Pattern_h': Pattern_h,
        'Pattern_w': Pattern_w,
        'SquareLength': SquareLength,
    }
    fetch(`http://${SlaveAdress}/setQualitySettings`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data)
    })
        .then(utils.HandleError)

    fetch(`/changeQualitySettings`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data)
    })
        .then(utils.HandleError)
}
//@ts-ignore
window.setReferencingSettings = setReferencingSettings;


function setAdvancedSettings() {
    //@ts-ignore
    let ColorThresholdMin = document.getElementById("id_ColorThresholdMin").value * 1;
    //@ts-ignore
    let ColorThresholdMax = document.getElementById("id_ColorThresholdMax").value * 1;
    //@ts-ignore
    let RefineAlignWithCAD = document.getElementById("id_RefineAlignWithCAD").checked;
    //@ts-ignore
    let UseVoxelGrid = document.getElementById("id_UseVoxelGrid").checked;


    let cameradata = {
        'ColorThresholdMin': ColorThresholdMin,
        'ColorThresholdMax': ColorThresholdMax,
        'serialnumber': G_CurrentCamera,
    }

    //Python
    fetch(`http://${SlaveAdress}/setCameraSettings`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(cameradata)
    })
        .then(utils.HandleError)

    //Database
    fetch(`/changeCameraSettings`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(cameradata)
    })
        .then(utils.HandleError)

    let qualitydata = {
        'RefineAlignWithCAD': RefineAlignWithCAD,
        'UseVoxelGrid': UseVoxelGrid,
    }
    //Python
    fetch(`http://${SlaveAdress}/setQualitySettings`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(qualitydata)
    })
        .then(utils.HandleError)
    //Database
    fetch(`/changeQualitySettings`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(qualitydata)
    })
        .then(utils.HandleError)
}
//@ts-ignore
window.setAdvancedSettings = setAdvancedSettings;


async function AdvancedButtonFunc() {

    let serials = await fetch('/getCamsOfCurrentSystem')
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            let serials = json['serialnumbers']
            $("#id_ThresholdCameraSelect").empty()
            let CameraSelect = document.getElementById('id_ThresholdCameraSelect')
            serials.forEach(serialnumber => {
                let newOption = document.createElement('option')
                newOption.text = serialnumber
                CameraSelect.appendChild(newOption)
            });
            $("#id_ThresholdCameraSelect").children().first().selected = true
            ThresholdsetCurrentCamera();
            return serials;
        })

    fetch(`http://${SlaveAdress}/getQualitySettings`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            //@ts-ignore
            document.getElementById("id_RefineAlignWithCAD").checked = json['RefineAlignWithCAD'];
            //@ts-ignore
            document.getElementById("id_UseVoxelGrid").checked = json['UseVoxelGrid'];
        })
    fetch(`http://${SlaveAdress}/getCameraSettings`,
        {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ 'serialnumber': G_CurrentCamera })
        })
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            //@ts-ignore
            document.getElementById("id_ColorThresholdMin").value = json['ColorThresholdMin'];
            //@ts-ignore
            document.getElementById("id_ColorThresholdMax").value = json['ColorThresholdMax'];
            utils.updateValueLabels(json['ColorThresholdMin'], "id_ColorThresholdMin");
            utils.updateValueLabels(json['ColorThresholdMax'], "id_ColorThresholdMax");
        })

}
//@ts-ignore
window.AdvancedButtonFunc = AdvancedButtonFunc;

async function SelectModelButtonFunc() {
    $.blockUI({ message: `<h1> <img src=${G_BLOCKUI_IMG_PATH} alt=''/> Processing model...</h1>` });
    utils.toggleMessage('id_ModelProgress');
    Renderer.LoadNewModelinRenderer();
    let select = document.getElementById('id_3DModelList');
    try {
        //@ts-ignore
        let filename = select.options[select.selectedIndex].text;

        await fetch("/changeQualitySettings", {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ "filename": filename })
        })
            .then(utils.HandleError)

        await utils.sendModelToWorkhorse("Quality", filename)
        utils.toggleMessage('id_SelectSuccess')
        $.unblockUI()
    }
    catch (error) { utils.toggleMessage(); $.unblockUI(); return; }
}
//@ts-ignore
window.SelectModelButtonFunc = SelectModelButtonFunc;