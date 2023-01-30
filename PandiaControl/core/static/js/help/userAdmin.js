import indexOf from '../../3rdpartylibs/jquery-main/src/var/indexOf.js';
import * as utils from '../../js/utils.js';

function changeAttributes(userid){
    console.log(userid);
    let newname= document.getElementById(`firstname-${userid}`).value
    let newmail= document.getElementById(`email-${userid}`).value
    if(validateEmail(newmail)== false){
        document.getElementById(`email-${userid}`).classList.add('is-invalid')
        return 
    }
    let formdata= {'newname': newname, 'newmail': newmail, 'id': userid}
    fetch('/changeUserData',
    {
        method: 'POST',
        body: JSON.stringify(formdata),
        headers: new Headers({
            "content-type": "application/json"
        })
    })
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (json) {
        if (json['Message'] == 'OK') {
            location.reload()
        }
        else {
            console.log("Unknown response from changeUserData");
        }
    })
}
window.changeAttributes= changeAttributes

function deleteUser(userid){
    if(confirm('Are you sure you want to delete this User?'))
    {
        fetch('/deleteUser',
        {
            method: 'POST',
            body: userid,
            headers: new Headers({
                "content-type": "text/plain"
            })
        })
        .then(utils.HandleError)
        .then(response => response.json())
        .then(function (json) {
            if (json['Message']=='OK')
            {
                location.reload()
            }
            else if (json['Message'] == "Can't delete current user")
            {
                alert(json['Message']);
            }
            else {
                console.log("Unknown response from deleteUser");
            }
        })
    }
    else 
        return
}
window.deleteUser= deleteUser

function ValidateUserForm(){
    // document.getElementById('userPassword').value= ''
    let inputfields = document.querySelectorAll('#newUserForm input')
    let messages= document.getElementsByClassName("message")
    Array.from(messages).forEach(function (element) {element.style.visibility= 'hidden'});
    let erroroccured= false
    inputfields.forEach(function(element, idx) {
        element.classList.remove('is-invalid')
        if(element.value== ''){
            element.classList.add('is-invalid')
            document.getElementById('invalidInput').style.visibility= 'visible'
            erroroccured= true
        }
        if(idx==1){
            if(validateEmail(element.value)== false){
                element.classList.add('is-invalid')
                document.getElementById('invalidMailInput').style.visibility= 'visible'
                erroroccured= true
            }
        }
        if(idx==3){
            if(element.value != inputfields[idx-1].value){
                element.classList.add('is-invalid')
                inputfields[idx-1].classList.add('is-invalid')
                document.getElementById('passwordmatch').style.visibility= 'visible'
                erroroccured= true
            }
        }
    });
    if(erroroccured){
        return
    }
    else {
        let inputfields = document.querySelectorAll('#newUserForm input')
        let formdata = {}
        formdata['role']= document.getElementById('newrole').options[document.getElementById('newrole').selectedIndex].text
        inputfields.forEach(function(element, idx) {
            if(idx != 3){formdata[`${element.id}`]= element.value}
        })
        console.log(formdata);
        fetch('/registerNewUser',
        {
            method: 'POST',
            body: JSON.stringify(formdata),
            headers: new Headers({
                "content-type": "application/json"
            })
        })
        .then(utils.HandleError)
        .then(response => response.json)
        .then(function (json) {
            if(json['Message'] == 'OK') {
                location.reload()
            }
            else {
                console.log("Unknown response from registerNewUser")
            }
        }
    }
}

function NewUser(){
    let inputfields = document.querySelectorAll('#newUserForm input')
    let formdata = {}
    formdata['OldPas']= document.getElementById('OldPass').value
    formdata['role']= document.getElementById('newrole').options[document.getElementById('newrole').selectedIndex].text
    inputfields.forEach(function(element, idx) {
        if(idx != 3){formdata[`${element.id}`]= element.value}
    })
    console.log(formdata);
    fetch('/registerNewUser', {
        method: 'POST',
        body: JSON.stringify(formdata),
        headers: new Headers({
            "content-type": "application/json"
        })
    })
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (json) {
        if(json['Message'] == 'OK') {
            location.reload()
        }
        // else if (json['Message'] == "Password is invalid"){
        //     document.getElementById('invalidpassword').style.visibility= 'visible'
        // }
    })
}
window.NewUser= NewUser

function validateEmail(email) {
    var re = /^(([^<>()[\]\.,;:\s@\"]+(\.[^<>()[\]\.,;:\s@\"]+)*)|(\".+\"))@(([^<>()[\]\.,;:\s@\"]+\.)+[^<>()[\]\.,;:\s@\"]{2,})$/i;
    return re.test(email.toLowerCase());
}

window.ValidateUserForm= ValidateUserForm