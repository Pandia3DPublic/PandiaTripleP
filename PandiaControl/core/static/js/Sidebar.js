import * as utils from './utils.js'
import * as Renderer from './Renderer.js'
import * as THREE from '../3rdpartylibs/three.js-master/build/three.module.js'

$('.toggle-container').on('click', function () {
    $('.toggle-container,.container,.images-container').removeClass('active');
    let id = $(this).attr('id');
    $('#' + id + ',#' + id + '_container,#' + id +'_images-container').addClass('active');
    if (id != 'id_Referencing') {
        try {
            $('#id_RenderCanvas').removeAttr('hidden')
        } catch (error) { }
    }
    if (id != 'id_FOV') {
        try {
            $('#id_FOVImage').attr("style", "display: none")
        } catch (error) { }
    }
});

$('.infotext-show').on('click', function () {
    $(this).next().toggle();
});

$('.infotext-hide').on('click', function () {
    $(this).parent().hide();
});

export function setSidebarModeButtonActive() {
    //@ts-ignore
    if (J_SideBarMode == 'active')
    {
        document.getElementById('id_ActiveButton').classList.add("active");
    }
    //@ts-ignore
    else if (J_SideBarMode == 'setup')
    {
        document.getElementById('id_SetupButton').classList.add("active");
    }
    else
    {
        console.log("Unknown Sidebar Mode");
    }
}

export function setupFileUploadListener() {
    //@ts-ignore
    let form = document.getElementById("id_FileUploadForm")
    //@ts-ignore
    form.addEventListener('submit', function (event) {
        const select = document.getElementById('id_3DModelList');
        event.preventDefault();    // prevent page from refreshing
        //@ts-ignore
        const formData = new FormData(form);  // grab the data inside the form fields
        utils.toggleMessage("id_ModelProgress")
        fetch('/uploadReferenceModelFile',
            {   // assuming the backend is hosted on the same server
                method: 'POST',
                body: formData,
            })
            .then(utils.HandleError)
            .then(response => response.json())
            .then(function (json) {
                if (json["filename"] == '500') {
                    utils.toggleMessage("id_SubmitFailFilename")
                    return
                }
                if (json["filename"] == '501') {
                    utils.toggleMessage("id_SubmitFailModel")
                    return
                }
                if (json["filename"] == '502') {
                    utils.toggleMessage("id_SubmitFailFreeCAD")
                    return
                }
                let newFileName = true;
                //@ts-ignore
                Array.from(select.options).forEach(function (element) {
                    if (element.text == json["filename"]) {
                        utils.toggleMessage("id_SubmitFailDuplicate")
                        newFileName = false;
                    }
                });
                if (newFileName === true) {
                    let newOption = document.createElement("option")
                    newOption.text = json["filename"]
                    select.appendChild(newOption)
                    utils.toggleMessage("id_SubmitSuccess")
                }
                document.getElementById('id_ModelProgress').style.visibility = 'hidden'
            });
    });
}

export async function getAndShowFOVPcd()
{
    return fetch(`http://${SlaveAdress}/getCroppedPcd`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({'serialnumber': G_CurrentCamera})
    })
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (json) {
        if (json.hasOwnProperty('Message') && json['Message'] == 'Failed') {
            console.log('getCroppedPcd failed');
            return false;
        }
        const buffer = new THREE.BufferGeometry();
        const PointMaterial = new THREE.PointsMaterial({ size: G_PointSizeBig, vertexColors: true });
        const pcd = new THREE.Points(buffer, PointMaterial);

        buffer.setAttribute('position', new THREE.Float32BufferAttribute(json.points, 3));
        buffer.setAttribute('color', new THREE.Float32BufferAttribute(json.colors, 3));

        Renderer.Scene.remove(Renderer.FOVPcd)
        Renderer.setFOVPcd(pcd);
        Renderer.Scene.add(Renderer.FOVPcd);
        Renderer.Controls.target.set(0,0,-1200)
        Renderer.Controls.update()
        return true;
    })
}
//@ts-ignore
window.getAndShowFOVPcd = getAndShowFOVPcd;


export async function recordAndShowFOVPcd()
{
    return fetch(`http://${SlaveAdress}/recordAndGetCroppedPcd`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({'serialnumber': G_CurrentCamera})
    })
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (json) {
        if (json.hasOwnProperty('Message') && json['Message'] == 'Failed') {
            console.log('recordAndGetCroppedPcd failed');
            return false;
        }
        const buffer = new THREE.BufferGeometry();
        const PointMaterial = new THREE.PointsMaterial({ size: G_PointSizeBig, vertexColors: true });
        const pcd = new THREE.Points(buffer, PointMaterial);

        buffer.setAttribute('position', new THREE.Float32BufferAttribute(json.points, 3));
        buffer.setAttribute('color', new THREE.Float32BufferAttribute(json.colors, 3));

        Renderer.Scene.remove(Renderer.FOVPcd)
        Renderer.setFOVPcd(pcd);
        Renderer.Scene.add(Renderer.FOVPcd);
        Renderer.Controls.target.set(0,0,-1200)
        Renderer.Controls.update()
        return true;
    })
}
//@ts-ignore
window.recordAndShowFOVPcd = recordAndShowFOVPcd;


function ModelButtonFunc() {
    Renderer.removeAllModels();
    Renderer.Scene.add(Renderer.CurrentReferenceModel);
    Renderer.Controls.target.set(0,0,0)
    Renderer.Controls.update()
}
//@ts-ignore
window.ModelButtonFunc = ModelButtonFunc

$('#id_ReferenceModel').trigger('click');

function DeleteModelButtonFunc() {
    let filename;
    let select = document.getElementById('id_3DModelList');
    //@ts-ignore
    try { filename = select.options[select.selectedIndex].text; }
    catch (error) { return }

    if(filename == ''){
        return;
    }

    utils.toggleMessage('id_ModelProgress');
    fetch(`http://${SlaveAdress}/clearModels`)
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function () {
        fetch('/DeleteModel', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({'filename':filename})
        })
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function () {
            //@ts-ignore
            select.remove(select.selectedIndex)
            Renderer.Scene.remove(Renderer.CurrentReferenceModel)
            Renderer.setCurrentReferenceModel(new THREE.Object3D)
            utils.toggleMessage('id_DeleteModelSuccess')
        })
    });
   
}
//@ts-ignore
window.DeleteModelButtonFunc = DeleteModelButtonFunc;


export async function setupFOVSliders()
{
    return fetch(`http://${SlaveAdress}/getFOVValues`, 
    {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({'serialnumber': G_CurrentCamera})
    })
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (json) {
        //@ts-ignore
        document.getElementById("id_left").value = json['crop_left'];
        //@ts-ignore
        document.getElementById("id_right").value = json['crop_right'];
        //@ts-ignore
        document.getElementById("id_up").value = json['crop_top'];
        //@ts-ignore
        document.getElementById("id_down").value = json['crop_bottom'];
        //@ts-ignore
        document.getElementById("id_front").value = json['crop_depth_inv'];
        //@ts-ignore
        document.getElementById("id_rear").value = json['crop_depth'];
        
        //@ts-ignore
        utils.updateValueLabels(json['crop_left'], "id_left");
        utils.updateValueLabels(json['crop_right'], "id_right");
        utils.updateValueLabels(json['crop_top'], "id_up");
        utils.updateValueLabels(json['crop_bottom'], "id_down");
        utils.updateValueLabels(json['crop_depth_inv'], "id_front");
        utils.updateValueLabels(json['crop_depth'], "id_rear");

        try { 
            //@ts-ignore
            document.getElementById("id_ground").value = json['crop_ground_height']; 
            utils.updateValueLabels(json['crop_ground_height'], "id_ground")
        } catch (error) {}
        try { 
            //@ts-ignore
            document.getElementById("id_groundfine").value = json['crop_ground_height_fine']; 
            utils.updateValueLabels(json['crop_ground_height_fine'], "id_groundfine")
        } catch (error) {}
        return true;
    })
}

export async function FOVRecordButtonFunc()
{
    utils.toggleMessage('id_FOVProgress')
    let success = await recordAndShowFOVPcd();
    await getFOVImage();
    if (success)
        utils.toggleMessage();
    else
        utils.toggleMessage('id_FOVFail')
}
//@ts-ignore
window.FOVRecordButtonFunc = FOVRecordButtonFunc


export async function FOVSliderFunc()
{
    utils.toggleMessage('id_FOVProgress')
    //@ts-ignore
    let left = document.getElementById("id_left").value * 1; //multiply with 1 to convert string to number
    //@ts-ignore
    let right = document.getElementById("id_right").value * 1;
    //@ts-ignore
    let up = document.getElementById("id_up").value * 1;
    //@ts-ignore
    let down = document.getElementById("id_down").value * 1;
    //@ts-ignore
    let front = document.getElementById("id_front").value * 1;
    //@ts-ignore
    let rear = document.getElementById("id_rear").value * 1;

    let data = {
        'serialnumber' : G_CurrentCamera,
        'crop_top': up, 'crop_bottom': down, 'crop_left': left,
        'crop_right': right, 'crop_depth': rear, 'crop_depth_inv': front
    }
    try {
        //@ts-ignore
        let ground = document.getElementById("id_ground").value * 1;
        //@ts-ignore
        let groundfine = document.getElementById("id_groundfine").value * 1;
        data['crop_ground_height'] = ground;
        data['crop_ground_height_fine'] = groundfine;
    } catch (error) {}

    await fetch('/changeCameraSettings', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data)
    })
    .then(utils.HandleError)
    
    await fetch(`http://${SlaveAdress}/setFOVValues`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data)
    })
    .then(utils.HandleError)

    await getAndShowFOVPcd();
    await getFOVImage();
    utils.toggleMessage()
}
//@ts-ignore
window.FOVSliderFunc = FOVSliderFunc;

export async function getFOVImage() {
    //@ts-ignore
    document.getElementById('id_FOVImage').style.display = "flex";
    return fetch(`http://${SlaveAdress}/getFOVImage`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({'serialnumber': G_CurrentCamera})
    })
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (data) {
            if (data['image'] != 0) {
                //@ts-ignore
                document.getElementById('id_FOVImage').src = "data:image/jpg;charset=utf-8;base64," + data['image'];
            }
            else {
                //@ts-ignore
                document.getElementById('id_FOVImage').src = `${G_DEFAULT_IMG_PATH}`;
            }
            return true;
        })
}
//@ts-ignore
window.getFOVImage = getFOVImage;


async function StartReferenceScan()
{
    document.getElementById('id_AddReferenceScan')?.classList.add('disabled');
    document.getElementById('id_RevertLastReferenceScan')?.classList.add('disabled');
    document.getElementById('id_RefineReferenceScan')?.classList.add('disabled');
    utils.toggleMessage("id_ModelProgress");
    Renderer.removeAllModels()
    fetch(`http://${SlaveAdress}/startReferenceScan`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json.hasOwnProperty('Message') && json['Message'] == 'Failed') {
                console.log('startReferenceScan failed');
                utils.toggleMessage();
                return false;
            }
            const buffer = new THREE.BufferGeometry();
            const PointMaterial = new THREE.PointsMaterial({ size: G_PointSize, vertexColors: true });
            const pcd = new THREE.Points(buffer, PointMaterial);

            buffer.setAttribute('position', new THREE.Float32BufferAttribute(json.points, 3));
            buffer.setAttribute('color', new THREE.Float32BufferAttribute(json.colors, 3));

            Renderer.setCurrentReferenceModel(pcd);
            Renderer.CurrentReferenceModel.matrixAutoUpdate = false
            Renderer.CurrentReferenceModel.matrix = utils.getFlipMatrix()
            Renderer.CurrentReferenceModel.matrixWorldNeedsUpdate = true;
            Renderer.Scene.add(Renderer.CurrentReferenceModel);
            document.getElementById('id_AddReferenceScan')?.classList.remove('disabled');
            utils.toggleMessage();
        })
}
//@ts-ignore
window.StartReferenceScan = StartReferenceScan;


async function AddReferenceScan(params) {
    console.log("added reference scan");
    utils.toggleMessage("id_ModelProgress");
    fetch(`http://${SlaveAdress}/AddReferenceScan`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json.hasOwnProperty('Message') && json['Message'] == 'Failed') {
                console.log('AddReferenceScan failed');
                utils.toggleMessage();
                return false;
            }
            Renderer.removeAllModels()
            const buffer = new THREE.BufferGeometry();
            const PointMaterial = new THREE.PointsMaterial({ size: G_PointSize, vertexColors: true });
            const pcd = new THREE.Points(buffer, PointMaterial);

            buffer.setAttribute('position', new THREE.Float32BufferAttribute(json.points, 3));
            buffer.setAttribute('color', new THREE.Float32BufferAttribute(json.colors, 3));

            Renderer.setCurrentReferenceModel(pcd);
            Renderer.CurrentReferenceModel.matrixAutoUpdate = false
            Renderer.CurrentReferenceModel.matrix = utils.getFlipMatrix()
            Renderer.CurrentReferenceModel.matrixWorldNeedsUpdate = true;
            Renderer.Scene.add(Renderer.CurrentReferenceModel);
            utils.toggleMessage();
            document.getElementById('id_RevertLastReferenceScan')?.classList.remove('disabled');
            document.getElementById('id_RefineReferenceScan')?.classList.remove('disabled');
        })
}
//@ts-ignore
window.AddReferenceScan = AddReferenceScan;

async function RevertLastReferenceScan(params) {
    utils.toggleMessage("id_ModelProgress");
    fetch(`http://${SlaveAdress}/RevertLastReferenceScan`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json.hasOwnProperty('Message') && json['Message'] == 'Failed') {
                console.log('RevertLastReferenceScan failed');
                utils.toggleMessage();
                return false;
            }
            Renderer.removeAllModels()
            const buffer = new THREE.BufferGeometry();
            const PointMaterial = new THREE.PointsMaterial({ size: G_PointSize, vertexColors: true });
            const pcd = new THREE.Points(buffer, PointMaterial);

            buffer.setAttribute('position', new THREE.Float32BufferAttribute(json.points, 3));
            buffer.setAttribute('color', new THREE.Float32BufferAttribute(json.colors, 3));

            Renderer.setCurrentReferenceModel(pcd);
            Renderer.CurrentReferenceModel.matrixAutoUpdate = false
            Renderer.CurrentReferenceModel.matrix = utils.getFlipMatrix()
            Renderer.CurrentReferenceModel.matrixWorldNeedsUpdate = true;
            Renderer.Scene.add(Renderer.CurrentReferenceModel);
            utils.toggleMessage();
            document.getElementById('id_RevertLastReferenceScan')?.classList.add('disabled');
        })
}
//@ts-ignore
window.RevertLastReferenceScan = RevertLastReferenceScan;

async function RefineReferenceScan(params) {
    console.log("refine reference scan");
    utils.toggleMessage("id_ModelProgress");
    fetch(`http://${SlaveAdress}/RefineReferenceScan`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json.hasOwnProperty('Message') && json['Message'] == 'Failed') {
                console.log('RefineReferenceScan failed');
                utils.toggleMessage();
                return false;
            }
            Renderer.removeAllModels()
            const buffer = new THREE.BufferGeometry();
            const PointMaterial = new THREE.PointsMaterial({ size: G_PointSize, vertexColors: true });
            const pcd = new THREE.Points(buffer, PointMaterial);

            buffer.setAttribute('position', new THREE.Float32BufferAttribute(json.points, 3));
            buffer.setAttribute('color', new THREE.Float32BufferAttribute(json.colors, 3));

            Renderer.setCurrentReferenceModel(pcd);
            Renderer.CurrentReferenceModel.matrixAutoUpdate = false
            Renderer.CurrentReferenceModel.matrix = utils.getFlipMatrix()
            Renderer.CurrentReferenceModel.matrixWorldNeedsUpdate = true;
            Renderer.Scene.add(Renderer.CurrentReferenceModel);
            utils.toggleMessage();
            document.getElementById('id_RevertLastReferenceScan')?.classList.add('disabled');
        })
}

//@ts-ignore
window.RefineReferenceScan = RefineReferenceScan;

async function SaveReferenceScan()
{
    utils.toggleMessage("id_ModelProgress");
    let scanName = ''
    //@ts-ignore
    try { scanName = document.getElementById('id_ReferenceScanName').value} catch (error) {}
    fetch(`/saveReferenceScan`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({'name':scanName})
    })
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json.hasOwnProperty('Message')) {
                console.log('saveReferenceScan failed: ' + json["Message"]);
                utils.toggleMessage("id_SubmitFail")
                return false;
            }

            const select = document.getElementById('id_3DModelList');
            let newOption = document.createElement("option")
            newOption.text = json["filename"]
            select.appendChild(newOption)
            utils.toggleMessage("id_SubmitSuccess")
        })
}
//@ts-ignore
window.SaveReferenceScan = SaveReferenceScan;