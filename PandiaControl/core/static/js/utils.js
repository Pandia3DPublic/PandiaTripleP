import * as THREE from '../3rdpartylibs/three.js-master/build/three.module.js'


export async function sendModelToWorkhorse(app, filename) {
    return fetch(`/sendModelToWorkhorse/${app}`, {
        method: 'POST',
        headers: { 'Content-Type': 'text/plain' },
        body: filename
    })
    .then(HandleError)
    .then(response => response.json())
    .then(function(json) {
        if (json['Message'] == "Failed") {
            console.log("Receive 3D Model failed");
            return false
        }
        return true;
    })
}

export function getFlipMatrix() {
    let flip = new THREE.Matrix4()
    flip.set(1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1)
    return flip
}

export function getMatrix(m) {
    let matrix = new THREE.Matrix4()
    matrix.set(m[0][0], m[0][1], m[0][2], m[0][3],
        m[1][0], m[1][1], m[1][2], m[1][3],
        m[2][0], m[2][1], m[2][2], m[2][3],
        m[3][0], m[3][1], m[3][2], m[3][3])
    matrix.premultiply(getFlipMatrix())
    return matrix
}

// IMPORTANT: there is also a copy of these Error handling functions in VolumeUpdateWorker.js as we cannot import it there (web worker)
// check for all response errors
export function HandleError(response) {
    if (!response.ok) {
        //Python error
        if (response.status == 420) {
            window.alert(`Error occured (status ${response.status}). Please reload page and try again.`)
            throw new Error(`Error occured (status ${response.status})`)
        }
        //Misc error
        throw new Error(`Error occured in ${response.url} (status ${response.status})`);
    }
    return response;
}

export function toggleMessage(id, text = ''){
    let messages= document.getElementsByClassName("message")
    Array.from(messages).forEach(function (element) {
        //@ts-ignore
        element.style.visibility = "hidden"
        if(element.id == id)
        {
            //@ts-ignore
            element.style.visibility ="visible"
            if (text != '') {
                element.textContent = text
            }
        }
    });
}
//@ts-ignore
window.toggleMessage= toggleMessage

export function updateValueLabels(val, id) {
    let labelid = "#" + id + "_label"
    let text = $(labelid).text();
    text = text + ""; //stupid cast to string
    let unit = text.substr(text.lastIndexOf(' ') +1, text.length); 
    text = text.substr(0, text.indexOf(':') +1); 
    text = text + " " + val + " " + unit;
    $(labelid).html(text);
}
//@ts-ignore
window.updateValueLabels = updateValueLabels;


export function removeAllChildNodes(parent_id) {
    try {
        let parent = document.getElementById(parent_id)
        while (parent.firstChild) {
            parent.removeChild(parent.firstChild);
        }
    } catch (error) { }
}

export function printElapsedTime(startTime = new Date(), functionName = "") {
  let endTime = new Date();
  //@ts-ignore
  let timeDiff = (endTime - startTime) / 1000;
  console.log(timeDiff + " seconds " + functionName);
}