import * as utils from '../utils.js'

fillListAndCurrentSystemTexts()

//######## fill available applications select menu ###########
fetch('/getDBWorkhorses')
    .then(utils.HandleError)
    .then(resp => resp.json())
    .then(function (workhorses) {
        workhorses.forEach(workhorse => {
            fetch(`http://${workhorse.credentials}/isAlive`)
                .then(utils.HandleError)
                .then(function () {
                    let workhorseSelect = document.getElementById('id_WorkhorseSelect')
                    let option = document.createElement("option")
                    option.text = workhorse.credentials
                    workhorseSelect.appendChild(option)
                })
                .catch(function (error) {
                    console.log(`Workhorse ${workhorse.credentials} is not alive`);
                })
        })
    })

//######## fill Systems select menu ###########
//note this gets called on start page load
async function fillListAndCurrentSystemTexts(params) {
    let systems = []
    await fetch('/getDBSystems')
        .then(utils.HandleError)
        .then(resp => resp.json())
        .then(function (json) {
            systems = json["content"]
        })

    let currentSystemName = ''
    await fetch('/getCurrentSystem')
        .then(utils.HandleError)
        .then(resp => resp.json())
        .then(function (json) {
            if (json.hasOwnProperty("content")) {
                currentSystemName = json["content"]["name"]
            }
        })

    let systemSelect = document.getElementById('id_SystemSelect');
    let currentSysText = document.getElementById('id_CurrentSystemText');
    for (let system of systems) 
    {
        let option = document.createElement("option");
        option.value = system["name"];

        let serialNumbers = []
        await fetch(`/getSystemCameraNames/${system['name']}`)
            .then(utils.HandleError)
            .then(resp => resp.json())
            .then(function (json) {
                if (json.hasOwnProperty('Message')) {
                    return false;
                }
                
                for (let cameraName of json['content']) {
                    let serial = cameraName.split(' : ')[1]
                    serialNumbers.push(serial)
                }
            })

        await fetch(`http://${system['credentials']}/isActive`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({'serialnumbers': serialNumbers})
        })
            .then(utils.HandleError)
            .then(resp => resp.json())
            .then(function (json) {
                if (json['Message'] == true) {
                    option.text = option.value + " - Active"
                }
                else{
                    option.text = option.value + " - Inactive"
                }
            })
            .catch(function () {
                option.text = option.value + " - Inactive"
            })
        systemSelect.appendChild(option)
        if (currentSystemName == system["name"]) {
            option.selected = true;
            currentSysText.textContent = option.text
            $( "#id_SystemSelect" ).trigger( "click" );
        }
    }

    //if no current system is set, auto select first item in system list
    if (currentSystemName == '')
    {
        if (systemSelect.options.length > 0)
        {
            let optionToSelect = systemSelect.options[0]
            optionToSelect.select = true
            currentSysText.textContent = optionToSelect.text
            $( "#id_SystemSelect" ).trigger( "click" );
        }
    }
}



function showAvailableCams() {
    document.getElementById('id_DetectingCamerasInfo').style.visibility = 'visible'
    let select = document.getElementById('id_WorkhorseSelect')
    let credentials;
    try {
        credentials = select.options[select.selectedIndex].text;
    } catch (error) {
        document.getElementById('id_DetectingCamerasInfo').style.visibility = 'hidden'
        return
    }
    if (credentials == 'Select your option') {
        document.getElementById('id_DetectingCamerasInfo').style.visibility = 'hidden'
        return;
    }
    fetch('http://' + credentials + '/isAlive')
        .then(utils.HandleError)
        .then(function () {
            fetch('http://' + credentials + '/getAvailableCameras')
                .then(utils.HandleError)
                .then(response => response.json())
                .then(function (json) {
                    document.getElementById('id_DetectingCamerasInfo').style.visibility = 'hidden'
                    $("#id_AvailableCamerasSelect").find('option').not(':disabled').remove()
                    if (json.length == 0) {
                        document.getElementById('id_NoCamerasInfo').style.visibility = 'visible'
                        return
                    }
                    else {
                        let CamSelect = document.getElementById('id_AvailableCamerasSelect')
                        CamSelect.disabled = false
                        document.getElementById('id_AddedCamerasList').disabled = false
                        let content = json['content']
                        content.forEach(array => {
                            let serialnumbers = array[0]
                            let cameratype = array[1]
                            serialnumbers.forEach(serialnumber => {
                                let newopt = document.createElement('option')
                                newopt.text = cameratype + " : " + serialnumber
                                CamSelect.appendChild(newopt)
                            });
                        });
                        // json.forEach(element => {
                        //     let newopt = document.createElement('option')
                        //     newopt.text = element
                        //     CamSelect.appendChild(newopt)
                        // });
                    }
                })
        })
        .catch(function (err) {
            document.getElementById('id_DetectingCamerasInfo').style.visibility = 'hidden'
            return
        })
}

//todo
//if connect fails display "image not availble"
function getPreviewImg() {
    function releaseGUIFunc() {
        $('select').attr('disabled', false);
        $('input').attr('disabled', false);
        document.getElementById('id_PreviewImageInfo').style.visibility = 'hidden'
    }
    $('select').attr('disabled', true);
    $('input').attr('disabled', true);
    document.getElementById('id_PreviewImageInfo').style.visibility = 'visible'

    let cameraSelect = document.getElementById('id_AvailableCamerasSelect');
    let cameraName;
    try { cameraName = cameraSelect.options[cameraSelect.selectedIndex].text; }
    catch (error) {
        releaseGUIFunc()
        return
    }
    let credentials;
    let workhorseSelect = document.getElementById("id_WorkhorseSelect")
    try {
        credentials = workhorseSelect.options[workhorseSelect.selectedIndex].text;
    } catch (error) {
        releaseGUIFunc()
        return
    }

    fetch(`http://${credentials}/getPreviewImg`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({'cameraName': cameraName})
    })
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (data) {
            if (data['image'] != 0) {
                document.getElementById('id_PreviewImage').src = "data:image/jpg;charset=utf-8;base64," + data['image'];
            }
            else {
                document.getElementById('id_PreviewImage').src = `${G_DEFAULT_IMG_PATH}`;
            }
            releaseGUIFunc()
        })
}

function AddCamera() {
    let newOption = true;
    let AvailableCams = document.getElementById('id_AvailableCamerasSelect')
    let AddedCameraList = document.getElementById('id_AddedCamerasList')
    let SelectedCamText
    try {
        let SelectedCam = AvailableCams.options[AvailableCams.selectedIndex]
        if (SelectedCam.disabled)
            return;
        SelectedCamText = SelectedCam.text;
    } catch (error) {
        return;
    }
    Array.from(AddedCameraList.options).forEach(function (element) {
        if (element.text == SelectedCamText) {
            console.log("Camera already in List");
            newOption = false;
        }
    });
    if (newOption === true) {
        let option = document.createElement("option")
        option.text = SelectedCamText
        AddedCameraList.appendChild(option)
    }
}

function RemoveCamera() {
    let CamList = document.getElementById('id_AddedCamerasList')
    let AvailableCams = document.getElementById('id_AvailableCamerasSelect')
    let SelectedCamText
    try {
        SelectedCamText = AvailableCams.options[AvailableCams.selectedIndex].text;
    } catch (error) {
        return
    }
    Array.from(CamList.options).forEach(function (element) {
        if (element.text == SelectedCamText) {
            CamList.removeChild(element)
        }
    });
}


function DeleteSystem() {
    let systemSelect = document.getElementById('id_SystemSelect')
    let systemName = ''
    try {
        systemName = systemSelect.options[systemSelect.selectedIndex].value
    } catch (error) {
        return;
    }
    if (!confirm('Are you sure that you want to delete the system?'))
    {
        return;
    }
    $.blockUI({ message: `<h1> <img src=${G_BLOCKUI_IMG_PATH} alt=''/> Deleting system...</h1>` });
    fetch('/DeleteSystem', {
        method: "POST",
        body: systemName,
        headers: new Headers({
            "content-type": "text/plain"
        })
    })
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json.hasOwnProperty('Message')) { //error case
                console.log(json['Message'])
                $.unblockUI()
                return;
            }
            fetch(`http://${json['credentials']}/clearCameraList`)
                .then(utils.HandleError)
                .then(function () { location.reload() })
                .catch(function () { location.reload() })
        })
        .catch(function(){
            $.unblockUI()
        })
}

async function addNewSystem() {
    let whSelect = document.getElementById("id_WorkhorseSelect");
    let whAdress;
    try {
        whAdress = whSelect.options[whSelect.selectedIndex].text;
    } catch (error) {
        return;
    }
    let systemCams = [...document.getElementById("id_AddedCamerasList").options].map(opt => opt.text)
    let systemName = document.getElementById("id_SystemName").value;

    if (systemName == '' || systemCams.length == 0 || whAdress == '') {
        alert('Error: One or more required fields are empty. Please check again and retry!')
        return
    }
    var data = {
        credentials: whAdress,
        cams: systemCams,
        name: systemName
    }
    if (!window.confirm(`Do you really want to create a new system with this setup?\n
                        Application: ${whAdress}\n
                        Cameras: ${systemCams}\n
                        Name: ${systemName}`)) {
        return;
    }

    let success = false;
    success = await fetch('/addNewSystemToDB', {
        method: "POST",
        body: JSON.stringify(data),
        headers: new Headers({
            "content-type": "application/json"
        })
    })
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json['Message'] == 'OK') {
                return true;
            }
            else {
                console.log("Unknown response from addNewSystemToDB");
                return false;
            }
        })

    success = await fetch(`/SetCurrentSystem/${systemName}`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json.hasOwnProperty('Message') && json['Message'] == false) {
                console.log("Set current system failed");
                return false;
            }
            else {
                SlaveAdress = json['SlaveAdress']
                return true;
            }
        })
    
    if (success)
        location.reload()
    else{
        console.log("addNewSystem failed");
    }
}

async function SystemSelectFunction(params) {
    utils.toggleMessage()
    let systemSelect = document.getElementById('id_SystemSelect');
    let systemCameraList = document.getElementById('id_SystemCamerasList')
    utils.removeAllChildNodes("id_SystemCamerasList")
    document.getElementById('id_SystemCamerasListLabel').innerHTML = 'Cameras of the selected System'
    let systemName = "undefined";
    try {
        systemName = systemSelect.options[systemSelect.selectedIndex].value;
    } catch (error) { 
        return;
    }

    //change the current user system in controller db
    let success = true;
    success = await fetch(`/SetCurrentSystem/${systemName}`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json.hasOwnProperty('Message') && json['Message'] == false) {
                return false;
            }
            SlaveAdress = json['SlaveAdress']
            return true;
        })
    if (!success) {
        console.log("System does not exist");
        return;
    }

    //query cameras of system in db

    let cameraNames;
    success = await fetch(`/getSystemCameraNames/${systemName}`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json.hasOwnProperty("Message")) {
                return false;
            }
            cameraNames = json["content"];
            document.getElementById('id_SystemCamerasListLabel').innerHTML += (" (" + SlaveAdress + ")")
            cameraNames.forEach(cameraName => {
                let newOption = document.createElement('option')
                newOption.text = cameraName
                newOption.disabled = true
                systemCameraList.appendChild(newOption)
            });
            return true;
        })

    // update the selector in the top right
    let currentSysText = document.getElementById('id_CurrentSystemText');
    // @ts-ignore
    currentSysText.textContent = systemSelect.options[systemSelect.selectedIndex].text;

    if (currentSysText.textContent.includes(" - Active")) {
        try { document.getElementById('nl2').classList.remove('disabled') } catch { }
        try { document.getElementById('nl3').classList.remove('disabled') } catch { }
    }
    else {
        try { document.getElementById('nl2').classList.add('disabled') } catch { }
        try { document.getElementById('nl3').classList.add('disabled') } catch { }
    }
}



function ActivateCurrentSystem(params) {
    if( $('#id_SystemSelect').has('option').length == 0 ) {
        return;
    }
    $.blockUI({ message: `<h1> <img src=${G_BLOCKUI_IMG_PATH} alt=''/> Activating system...</h1>` });
    utils.toggleMessage('id_ActivateSystemInfo')
    fetch(`/activateCurrentSystem`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            //failure case
            if (json.hasOwnProperty("Message") && json["Message"] != "OK") {
                console.log(json["Message"]);
                utils.toggleMessage('id_ErrorSystemInfo', json['Message'])
                $.unblockUI()
                return;
            }

            //success case
            // update the selector in the top right
            let currentSysText = document.getElementById('id_CurrentSystemText');
            let systemSelect = document.getElementById('id_SystemSelect');
            // @ts-ignore
            systemSelect.options[systemSelect.selectedIndex].text = systemSelect.options[systemSelect.selectedIndex].value + " - Active";
            // @ts-ignore
            currentSysText.textContent = systemSelect.options[systemSelect.selectedIndex].value + " - Active";

            if (currentSysText.textContent.includes(" - Active")) {
                try { document.getElementById('nl2').classList.remove('disabled') } catch { }
                try { document.getElementById('nl3').classList.remove('disabled') } catch { }
            }
            else {
                try { document.getElementById('nl2').classList.add('disabled') } catch { }
                try { document.getElementById('nl3').classList.add('disabled') } catch { }
            }

            utils.toggleMessage()
            $.unblockUI()
        })
        .catch(function() {
            $.unblockUI()
        })
}


function DeactivateCurrentSystem(params) {
    if( $('#id_SystemSelect').has('option').length == 0 ) {
        return;
    }
    $.blockUI({ message: `<h1> <img src=${G_BLOCKUI_IMG_PATH} alt=''/> Deactivating system...</h1>` });
    utils.toggleMessage('id_DeactivateSystemInfo')
    fetch(`/deactivateCurrentSystem`)
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json.hasOwnProperty("Message") && json['Message'] != 'OK') {
                console.log(json["Message"]);
                utils.toggleMessage()
                $.unblockUI()
                return;
            }
            // update the selector in the top right
            let currentSysText = document.getElementById('id_CurrentSystemText');
            let systemSelect = document.getElementById('id_SystemSelect');
            // @ts-ignore
            systemSelect.options[systemSelect.selectedIndex].text = systemSelect.options[systemSelect.selectedIndex].value + " - Inactive";
            // @ts-ignore
            currentSysText.textContent = systemSelect.options[systemSelect.selectedIndex].value + " - Inactive";

            if (currentSysText.textContent.includes(" - Active")) {
                try { document.getElementById('nl2').classList.remove('disabled') } catch { }
                try { document.getElementById('nl3').classList.remove('disabled') } catch { }
            }
            else {
                try { document.getElementById('nl2').classList.add('disabled') } catch { }
                try { document.getElementById('nl3').classList.add('disabled') } catch { }
            }
            
            utils.toggleMessage()
            $.unblockUI()
        })
        .catch(function() {
            $.unblockUI()
        })
}


window.SystemSelectFunction = SystemSelectFunction
window.ActivateCurrentSystem = ActivateCurrentSystem
window.DeactivateCurrentSystem = DeactivateCurrentSystem
window.addNewSystem = addNewSystem
window.getPreviewImg = getPreviewImg
window.showAvailableCams = showAvailableCams
window.RemoveCamera = RemoveCamera
window.AddCamera = AddCamera
window.DeleteSystem = DeleteSystem


