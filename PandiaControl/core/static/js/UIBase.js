
import * as utils from './utils.js'

// UIBaseStartSystem()

//gets called once on page load
// function UIBaseStartSystem() {
//     utils.StartCurrentSystem();
// }


fillCurrentSystemText()

// this gets called on every page load
async function fillCurrentSystemText(params) {
    let currentSysText = document.getElementById('id_CurrentSystemText');
    if (currentSysText.textContent.includes(" - Active")) {
        try { document.getElementById('nl2').classList.remove('disabled') } catch { }
        try { document.getElementById('nl3').classList.remove('disabled') } catch { }
    }
    else {
        try { document.getElementById('nl2').classList.add('disabled') } catch { }
        try { document.getElementById('nl3').classList.add('disabled') } catch { }
    }
    let currentSystemName = ''
    let syscredentials = ''
    let success = true
    success = await fetch('/getCurrentSystem')
        .then(utils.HandleError)
        .then(resp => resp.json())
        .then(function (json) {
            if (json.hasOwnProperty("content")) {
                currentSystemName = json["content"]["name"]
                syscredentials = json["content"]["credentials"]
                return true;
            }
            else
                return false
        })
    if (!success) {return success}

    let serialNumbers = []
    success = await fetch(`/getSystemCameraNames/${currentSystemName}`)
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
            return true;
        })
    if (!success) { return success }
    

    await fetch(`http://${syscredentials}/isActive`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({'serialnumbers': serialNumbers})
    })
        .then(utils.HandleError)
        .then(resp => resp.json())
        .then(function (json) {
            if (json['Message']) {
                currentSysText.textContent = currentSystemName + " - Active"
            }
            else {
                currentSysText.textContent = currentSystemName + " - Inactive"
            }
        })
        .catch(function () {
            currentSysText.textContent = currentSystemName + " - Inactive"
        })

    if (currentSysText.textContent.includes(" - Active")) {
        try { document.getElementById('nl2').classList.remove('disabled') } catch { }
        try { document.getElementById('nl3').classList.remove('disabled') } catch { }
    }
    else {
        try { document.getElementById('nl2').classList.add('disabled') } catch { }
        try { document.getElementById('nl3').classList.add('disabled') } catch { }

        //redirect if current system is not active (or set) and we are on quality or volume
        if (/QualityControl|VolumeScanner/.test(window.location.href))
        {
            window.location.href = window.location.origin;
        }

    }
}
