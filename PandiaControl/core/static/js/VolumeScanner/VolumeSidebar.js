import * as Renderer from '../Renderer.js'
import * as utils from '../utils.js'
import * as THREE from '../../3rdpartylibs/three.js-master/build/three.module.js'
import { setupFOVSliders, recordAndShowFOVPcd, getFOVImage } from '../Sidebar.js'

// time in ms
function delay(time) {
    return new Promise(resolve => setTimeout(resolve, time));
}

function setPositionValues() {
    utils.toggleMessage('id_CalibrationProgress')
    //@ts-ignore
    let xposition = document.getElementById("id_xposition").value * 1; //multiply with 1 to convert string to number
    //@ts-ignore
    let yposition = document.getElementById("id_yposition").value * 1;
    //@ts-ignore
    let zposition = document.getElementById("id_zposition").value * 1;
    //@ts-ignore
    let xrotation = document.getElementById("id_xrotation").value * 1;
    //@ts-ignore
    let yrotation = document.getElementById("id_yrotation").value * 1;
    //@ts-ignore
    let zrotation = document.getElementById("id_zrotation").value * 1;

    let data = {
        'xposition': xposition, 'yposition': yposition, 'zposition': zposition,
        'xrotation': xrotation, 'yrotation': yrotation, 'zrotation': zrotation
    }
    fetch('/changeVolumeSettings', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data)
    })
        .then(utils.HandleError)

    fetch(`http://${SlaveAdress}/setPositionValues`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data)
    })
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            Renderer.CurrentReferenceModel.matrix = utils.getMatrix(json['transformation'])
            Renderer.CurrentReferenceModel.matrixWorldNeedsUpdate = true;
            utils.toggleMessage();
        });

}
//@ts-ignore
window.setPositionValues = setPositionValues;

function setMiscSettings() {
    //@ts-ignore
    let VolumeNoise = document.getElementById("id_VolumeNoise").value * -1;
    //@ts-ignore
    let DensityFactor = document.getElementById("id_DensityFactor").value * 1;
    if (DensityFactor < 0)
        DensityFactor = 0
    //@ts-ignore
    document.getElementById("id_DensityFactor").value = DensityFactor
    //@ts-ignore
    let MaxProductHeight = document.getElementById("id_MaxProductHeight").value * 1;
    //@ts-ignore
    let ProductPresenceThreshold = document.getElementById("id_ProductPresenceThreshold").value * 1;

    let data = {
        'VolumeNoise': VolumeNoise,
        'DensityFactor': DensityFactor,
        'MaxProductHeight': MaxProductHeight,
        'ProductPresenceThreshold': ProductPresenceThreshold
    }
    //this does database
    fetch('/changeVolumeSettings', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data)
    })
        .then(utils.HandleError)

    //this does c++
    fetch(`http://${SlaveAdress}/setVolumeSettings`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data)
    })
        .then(utils.HandleError)
}
//@ts-ignore
window.setMiscSettings = setMiscSettings;


function setBeltSettings() {
    //@ts-ignore
    let BeltSpeed = document.getElementById("id_BeltSpeed").value * 1;
    //@ts-ignore
    let UseBeltSpeedEndpoint = document.getElementById("id_UseBeltSpeedEndpoint").checked;
    //@ts-ignore
    let BeltSpeedEndpointAddress = document.getElementById("id_BeltSpeedEndpointAddress").value;
    //@ts-ignore
    let BeltSpeedEndpointPayload = document.getElementById("id_BeltSpeedEndpointPayload").value;
    //@ts-ignore
    let BeltSpeedEndpointDistance = document.getElementById("id_BeltSpeedEndpointDistance").value * 1;
    //@ts-ignore
    let BeltSpeedEndpointUsername = document.getElementById("id_BeltSpeedEndpointUsername").value;
    //@ts-ignore
    let BeltSpeedEndpointPassword = document.getElementById("id_BeltSpeedEndpointPassword").value;

    let data = { 
        "BeltSpeed": BeltSpeed,
        "UseBeltSpeedEndpoint": UseBeltSpeedEndpoint,
        "BeltSpeedEndpointAddress": BeltSpeedEndpointAddress,
        "BeltSpeedEndpointPayload": BeltSpeedEndpointPayload,
        "BeltSpeedEndpointDistance": BeltSpeedEndpointDistance,
        "BeltSpeedEndpointUsername": BeltSpeedEndpointUsername,
        "BeltSpeedEndpointPassword": BeltSpeedEndpointPassword,
    }
    //this does database 
    fetch('/changeVolumeSettings', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data)
    })
        .then(utils.HandleError)

    //this does c++
    fetch(`http://${SlaveAdress}/setVolumeSettings`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data)
    })
        .then(utils.HandleError)
}
//@ts-ignore
window.setBeltSettings = setBeltSettings;

async function TestEndpointButtonFunc() {
    utils.toggleMessage('id_BeltProgress')
    fetch(`http://${SlaveAdress}/TestEndpointButtonFunc`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ "MaxWaitTime": 5 })
    })
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json.hasOwnProperty('BeltSpeed'))
                utils.toggleMessage('id_BeltEndpointSuccess', "Success! BeltSpeed is " + json['BeltSpeed'] + " m/s");
            else if (json.hasOwnProperty('Error'))
                utils.toggleMessage('id_BeltEndpointFail', json['Error'])
            else
                utils.toggleMessage('id_BeltEndpointFail')
        })
        .catch(function (error) {
            utils.toggleMessage('id_BeltFail')
            console.log("Failed to access workhorse!");
            console.log(error);
        })
}
//@ts-ignore
window.TestEndpointButtonFunc = TestEndpointButtonFunc;


// see https://stackoverflow.com/questions/46946380/fetch-api-request-timeout/57888548#57888548
const fetchTimeout = (url, ms, { signal, ...options } = {}) => {
    const controller = new AbortController();
    const promise = fetch(url, { signal: controller.signal, ...options });
    if (signal) signal.addEventListener("abort", () => controller.abort());
    const timeout = setTimeout(() => controller.abort(), ms);
    return promise.finally(() => clearTimeout(timeout));
};


function TestEndpointButtonFunc_JS() {
    utils.toggleMessage('id_BeltProgress')
    //@ts-ignore
    let BeltSpeedEndpointAddress = document.getElementById("id_BeltSpeedEndpointAddress").value;
    //@ts-ignore
    let BeltSpeedEndpointPayload = document.getElementById("id_BeltSpeedEndpointPayload").value;
    //@ts-ignore
    let BeltSpeedEndpointDistance = document.getElementById("id_BeltSpeedEndpointDistance").value;
    //@ts-ignore
    let BeltSpeedEndpointUsername = document.getElementById("id_BeltSpeedEndpointUsername").value;
    //@ts-ignore
    let BeltSpeedEndpointPassword = document.getElementById("id_BeltSpeedEndpointPassword").value;

    fetchTimeout(`http://${BeltSpeedEndpointAddress}`, 2000 ,{
        method: 'POST',
        headers: {
            'Authorization': 'Basic ' + btoa(BeltSpeedEndpointUsername + ':' + BeltSpeedEndpointPassword), 
            'Content-Type': 'application/json' 
        },
        body: '["' + BeltSpeedEndpointPayload + '"]'
    })
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (json) {
        console.log("Handling response")
        if (!json.hasOwnProperty('readResults'))
        {
            utils.toggleMessage('id_BeltEndpointFailJsonResults')
            return;
        }
        let idx = -1;
        for (var i = 0; i < json['readResults'].length; i++) {

            if (json['readResults'][i]['id'] == BeltSpeedEndpointPayload)
            {
                idx = i;
            }
        }
        if (idx == -1)
        {
            utils.toggleMessage('id_BeltEndpointFailPayload')
            return;
        }
        if (!(json['readResults'][idx].hasOwnProperty('v') && json['readResults'][idx].hasOwnProperty('t')))
        {
            utils.toggleMessage('id_BeltEndpointFailJsonSpeed')
            return;
        }
        utils.toggleMessage('id_BeltEndpointSuccess', "Success! v: " + json['readResults'][idx]['v'] + " t: " + json['readResults'][idx]['t'])
    })
    .catch(function (error) {
        utils.toggleMessage('id_BeltEndpointFail')
        console.log("Failed to access endpoint!");
        console.log(error);
    })
}
//@ts-ignore
window.TestEndpointButtonFunc_JS = TestEndpointButtonFunc_JS;


async function FOVButtonFunc() {
    utils.toggleMessage('id_FOVProgress')
    Renderer.removeAllModels();
    Renderer.Scene.add(Renderer.CameraModel)

    await setupFOVSliders();
    await recordAndShowFOVPcd();
    await getFOVImage();
    utils.toggleMessage()
}
//@ts-ignore
window.FOVButtonFunc = FOVButtonFunc

export async function CalibrationRecordButtonFunc()
{
    utils.toggleMessage('id_CalibrationProgress')
    let success = await recordAndShowFOVPcd();
    if (success)
        utils.toggleMessage();
    else
        utils.toggleMessage('id_CalibrationFail')
}
//@ts-ignore
window.CalibrationRecordButtonFunc = CalibrationRecordButtonFunc

async function CalibrationButtonFunc() {
    utils.toggleMessage('id_CalibrationProgress')
    Renderer.removeAllModels()
    Renderer.Scene.add(Renderer.CurrentReferenceModel)
    Renderer.Scene.add(Renderer.CameraModel)
    recordAndShowFOVPcd();

    fetch(`http://${SlaveAdress}/readCalibrationData`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            Renderer.CurrentReferenceModel.matrix = utils.getMatrix(json['transformation'])
            Renderer.CurrentReferenceModel.matrixWorldNeedsUpdate = true;

            //@ts-ignore
            document.getElementById("id_xposition").value = json['x_offset'];
            //@ts-ignore
            document.getElementById("id_yposition").value = json['y_offset'];
            //@ts-ignore
            document.getElementById("id_zposition").value = json['z_offset'];
            //@ts-ignore
            document.getElementById("id_xrotation").value = json['alpha'];
            //@ts-ignore
            document.getElementById("id_yrotation").value = json['beta'];
            //@ts-ignore
            document.getElementById("id_zrotation").value = json['gamma'];
            utils.updateValueLabels(json['x_offset'], "id_xposition")
            utils.updateValueLabels(json['y_offset'], "id_yposition")
            utils.updateValueLabels(json['z_offset'], "id_zposition")
            utils.updateValueLabels(json['alpha'], "id_xrotation")
            utils.updateValueLabels(json['beta'], "id_yrotation")
            utils.updateValueLabels(json['gamma'], "id_zrotation")

            utils.toggleMessage()
        });
}
//@ts-ignore
window.CalibrationButtonFunc = CalibrationButtonFunc;

function BeltCalibrationButtonFunc(params) {
    utils.toggleMessage('id_BeltProgress')
    document.getElementById('id_BeltSecondButton')?.classList.add('disabled');
    fetch(`http://${SlaveAdress}/getVolumeSettings`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            //@ts-ignore
            document.getElementById("id_BeltSpeed").value = json['BeltSpeed'];
            //@ts-ignore
            document.getElementById("id_UseBeltSpeedEndpoint").checked = json['UseBeltSpeedEndpoint'];
            //@ts-ignore
            document.getElementById("id_BeltSpeedEndpointAddress").value = json['BeltSpeedEndpointAddress'];
            //@ts-ignore
            document.getElementById("id_BeltSpeedEndpointPayload").value = json['BeltSpeedEndpointPayload'];
            //@ts-ignore
            document.getElementById("id_BeltSpeedEndpointDistance").value = json['BeltSpeedEndpointDistance'];
            //@ts-ignore
            document.getElementById("id_BeltSpeedEndpointUsername").value = json['BeltSpeedEndpointUsername'];
            //@ts-ignore
            document.getElementById("id_BeltSpeedEndpointPassword").value = json['BeltSpeedEndpointPassword'];

            utils.toggleMessage()
        });

    // utils.toggleMessage('id_BeltProgress')
    Renderer.removeAllModels();
    Renderer.Scene.add(Renderer.CameraModel)
}
//@ts-ignore
window.BeltCalibrationButtonFunc = BeltCalibrationButtonFunc;

function recordAndReturnFirstBeltPcd(params) {
    utils.toggleMessage('id_BeltProgress')
    Renderer.removeAllModels();
    Renderer.Scene.add(Renderer.CameraModel)

    return fetch(`http://${SlaveAdress}/recordAndReturnFirstBeltPcd`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ 'serialnumber': G_CurrentCamera })
    })
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json.hasOwnProperty('Message') && json['Message'] == 'Failed') {
                console.log('recordAndGetCroppedPcd failed');
                utils.toggleMessage('id_BeltFail')
                return false;
            }
            const buffer = new THREE.BufferGeometry();
            const PointMaterial = new THREE.PointsMaterial({ size: G_PointSizeBig, vertexColors: true });
            const pcd = new THREE.Points(buffer, PointMaterial);

            buffer.setAttribute('position', new THREE.Float32BufferAttribute(json.points, 3));
            buffer.setAttribute('color', new THREE.Float32BufferAttribute(json.colors, 3));

            Renderer.Scene.remove(Renderer.FirstBeltPcd)
            Renderer.setFirstBeltPcd(pcd);
            Renderer.Scene.add(Renderer.FirstBeltPcd);
            Renderer.Controls.target.set(0, 0, -1200)
            Renderer.Controls.update()

            document.getElementById('id_BeltSecondButton')?.classList.remove('disabled');
            utils.toggleMessage()
            return true;
        })
}
//@ts-ignore
window.recordAndReturnFirstBeltPcd = recordAndReturnFirstBeltPcd;


async function recordAndReturnSecondBeltPcd(params) {
    utils.toggleMessage('id_BeltProgress')
    let success = true;
    success = await fetch(`http://${SlaveAdress}/recordAndReturnSecondBeltPcd`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ 'serialnumber': G_CurrentCamera })
    })
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json.hasOwnProperty('Message') && json['Message'] == 'Failed') {
                console.log('recordAndReturnSecondBeltPcd failed');
                utils.toggleMessage('id_BeltFail')
                return false;
            }
            const buffer = new THREE.BufferGeometry();
            const PointMaterial = new THREE.PointsMaterial({ size: G_PointSizeBig, vertexColors: true });
            const pcd = new THREE.Points(buffer, PointMaterial);

            buffer.setAttribute('position', new THREE.Float32BufferAttribute(json.points, 3));
            buffer.setAttribute('color', new THREE.Float32BufferAttribute(json.colors, 3));

            Renderer.Scene.remove(Renderer.SecondBeltPcd)
            Renderer.setSecondBeltPcd(pcd);
            Renderer.Scene.add(Renderer.SecondBeltPcd);
            Renderer.Controls.target.set(0, 0, -1200)
            Renderer.Controls.update()
            return true;
        })
        if (!success) { console.log("recordAndReturnSecondBeltPcd failed"); return success; }
    

    let DirectionVector
    let BasePlanePoint
    //this does set the c++ DirectionVector variable
    success = await fetch(`http://${SlaveAdress}/calculateAndReturnVelocityVector`) 
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            //get vector from json
            //draw arrow with threejs

            BasePlanePoint = json["BasePlanePoint"]
            let jsondir = json["DirectionVector"]
            DirectionVector = new THREE.Vector3(jsondir[0], -jsondir[1], -jsondir[2]);
            // const dir = new THREE.Vector3(jsondir[0], jsondir[1], jsondir[2]);
            //normalize the direction vector (convert to vector of length 1)
            DirectionVector.normalize();
            const origin = new THREE.Vector3(0, 0, 0);
            const length = 1000;
            const hex = 0x00a86b;

            const arrowHelper = new THREE.ArrowHelper(DirectionVector, origin, length, hex);
            Renderer.Scene.remove(Renderer.DirectionArrow)
            Renderer.setDirectionArrow(arrowHelper);
            Renderer.Scene.add(Renderer.DirectionArrow);
            return true;
        });
        if (!success) { console.log("calculateAndReturnVelocityVector failed"); return success; }


    let dirstring = DirectionVector.x + "," + (-DirectionVector.y) + "," + (-DirectionVector.z);
    let data = { 'DirectionVector': dirstring, "BasePlanePoint": BasePlanePoint };
    console.log(data);
    //this does database 
    fetch('/changeVolumeSettings', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data)
    }).then(utils.HandleError)

    utils.toggleMessage('id_BeltSuccess');
}
//@ts-ignore
window.recordAndReturnSecondBeltPcd = recordAndReturnSecondBeltPcd;

function MiscButtonFunc() {
    fetch(`http://${SlaveAdress}/getVolumeSettings`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            //@ts-ignore
            document.getElementById("id_VolumeNoise").value = -json['VolumeNoise'];
            utils.updateValueLabels(-json['VolumeNoise'], "id_VolumeNoise")
            //@ts-ignore
            document.getElementById("id_DensityFactor").value = json['DensityFactor'];
            //@ts-ignore
            document.getElementById("id_MaxProductHeight").value = json['MaxProductHeight'];
            //@ts-ignore
            document.getElementById("id_ProductPresenceThreshold").value = json['ProductPresenceThreshold'];
        });
}
//@ts-ignore
window.MiscButtonFunc = MiscButtonFunc;



function CenterButtonFunc() {
    utils.toggleMessage('id_CalibrationProgress')
    fetch(`http://${SlaveAdress}/centerButton`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json.hasOwnProperty('Message'))
            {
                 utils.toggleMessage('id_CalibrationFail')
                 return;
            } 
            // Renderer.setTranslationOffset(json['offset'])
            Renderer.CurrentReferenceModel.matrix = utils.getMatrix(json['transformation'])
            Renderer.CurrentReferenceModel.matrixWorldNeedsUpdate = true;

            //@ts-ignore
            document.getElementById("id_xposition").value = 0;
            //@ts-ignore
            document.getElementById("id_yposition").value = 0;
            //@ts-ignore
            document.getElementById("id_zposition").value = 0;
            utils.updateValueLabels(0, "id_xposition")
            utils.updateValueLabels(0, "id_yposition")
            utils.updateValueLabels(0, "id_zposition")

            let data = {
                'TranslationOffset': json['newOffsets'],
                'TransMatrix': json['transformation'],
            }
            fetch('/changeVolumeSettings', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(data)
            })
                .then(utils.HandleError)
            
            utils.toggleMessage()
        })

}
//@ts-ignore
window.CenterButtonFunc = CenterButtonFunc


function ICPButtonFunc() {
    utils.toggleMessage('id_CalibrationProgress')
    fetch(`http://${SlaveAdress}/icpButton`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json.hasOwnProperty('Message')) {
                utils.toggleMessage('id_CalibrationFail')
                return;
            } 
            Renderer.CurrentReferenceModel.matrix = utils.getMatrix(json['transformation'])
            Renderer.CurrentReferenceModel.matrixWorldNeedsUpdate = true;
            fetch('/changeVolumeSettings', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ 'TransMatrix': json['transformation'] })
            })
                .then(utils.HandleError)

            utils.toggleMessage()
        })

}
//@ts-ignore
window.ICPButtonFunc = ICPButtonFunc

async function SelectModelButtonFunc() {
    $.blockUI({ message: `<h1> <img src=${G_BLOCKUI_IMG_PATH} alt=''/> Processing model...</h1>` });
    utils.toggleMessage('id_ModelProgress');
    Renderer.LoadNewModelinRenderer();
    let select = document.getElementById('id_3DModelList');
    try {
        //@ts-ignore
        let filename = select.options[select.selectedIndex].text;

        await fetch("/changeVolumeSettings", {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ "filename": filename })
        })
            .then(utils.HandleError)

        await utils.sendModelToWorkhorse("Volume", filename)
        utils.toggleMessage('id_SelectSuccess')
        $.unblockUI()
    }
    catch (error) { utils.toggleMessage(); $.unblockUI(); return; }
}
//@ts-ignore
window.SelectModelButtonFunc = SelectModelButtonFunc;