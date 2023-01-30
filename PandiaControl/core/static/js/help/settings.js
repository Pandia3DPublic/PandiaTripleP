import * as utils from '../utils.js';

function submit(submitFlag) {
    switch (submitFlag){
        case 1:
            let pwinput = document.getElementById("Inputpas");
            let pwinputconf = document.getElementById("InputpasConf");
            if (pwinput.value == pwinputconf.value) {
                document.getElementById("matchingerror").style.visibility = "hidden"
                pwinputconf.classList.remove("is-invalid")
                let requestData=  { newPas: pwinputconf.value}
                fetch('/help/settings/2/changePW', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(requestData)
                })
                .then(utils.HandleError)
                .then(response => response.json())
                .then(function(json) {
                    if (json['Message'] == 'OK') {
                        utils.toggleMessage('pwsuccess')
                    }
                    else {
                        console.log("Unknown response from changePW");
                    }
                })
            }
            else {
                utils.toggleMessage('matchingerror')
                pwinputconf.classList.add("is-invalid")
            }
            break;
        case 0:
            let emailinput = document.getElementById("Inputmail");
            if (validateEmail(emailinput.value) == true) {
                document.getElementById("mailerror").style.visibility = "hidden"
                emailinput.classList.remove("is-invalid")
                let requestData = { email: emailinput.value}
                fetch('/help/settings/2/changeEmail', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(requestData)
                })
                .then(utils.HandleError)
                .then(response => response.json())
                .then(function(json) {
                    if (json['Message'] == 'OK') {
                        utils.toggleMessage('mailsuccess')
                    }
                    else {
                        console.log("Unknown response from changeEmail");
                    }
                })
            }
            else {
                utils.toggleMessage('mailerror')
                emailinput.classList.add("is-invalid")
            }
            break;
    }
}
window.submit = submit

document.getElementById("Inputpas").oninput = () => {
    const actor = document.getElementById("Inputpas");
    const target1 = document.getElementById("InputpasConf");
    const target2 = document.getElementById("InputPasSub")
    const target3 = document.getElementById('clickableAwesomeFont')
    target3.style.visibility= 'visible'
    target2.classList.remove("disabled");
    if (actor.value.length == 0) {
        target2.classList.add("disabled")
    }
    if(actor.value.length == 0 && target1.value.length== 0){
        target3.style.visibility= 'hidden'
    }
    target1.disabled = false;
}

document.getElementById("Inputmail").oninput = () => {
    if (document.getElementById('Inputmail').value.length == 0 || document.getElementById('Inputmail').value == J_UserMail) {
        document.getElementById("InputmailCon").classList.add("disabled");
    }
    else {
        document.getElementById("InputmailCon").classList.remove("disabled")
    };
}

function validateEmail(email) {
    var re = /^(([^<>()[\]\.,;:\s@\"]+(\.[^<>()[\]\.,;:\s@\"]+)*)|(\".+\"))@(([^<>()[\]\.,;:\s@\"]+\.)+[^<>()[\]\.,;:\s@\"]{2,})$/i;
    return re.test(email.toLowerCase());
}

function showPassword() {
    document.getElementById("Inputpas").type = 'text';
    if (document.getElementById("InputpasConf").value.length != 0) {
        document.getElementById("InputpasConf").type = 'text';
    }
}
window.showPassword = showPassword

function hidePassword() {
    document.getElementById("Inputpas").type = 'password';
    if (document.getElementById("InputpasConf").value.length != 0) {
        document.getElementById("InputpasConf").type = 'password';
    }
}
window.hidePassword = hidePassword

