import * as utils from '../utils.js'

function confirmThreadStop(link) {
    fetch(`http://${SlaveAdress}/ThreadisRunning`)
    .then(utils.HandleError)
    .then(response => response.json())
    .then(function (json) {
        if (json['Message'] == 'OK'){
            if (confirm("Navigating to settings/system will stop the current volume measurement process. Do you want to continue?")){
                fetch(`http://${SlaveAdress}/stopVolumeThread`)
                .then(utils.HandleError)
                .then(function () {
                    window.location.href = link;
                })
            }
        } else {
            window.location.href = link;
        }
    })
}
// @ts-ignore
window.confirmThreadStop = confirmThreadStop;