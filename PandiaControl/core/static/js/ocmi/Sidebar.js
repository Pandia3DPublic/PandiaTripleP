function togglecon(param) {
    var b = document.getElementById("sidebarbutton" + param);
    var c = document.getElementById("container" + param);
    if (c.style.visibility == "visible") {
        c.style.visibility = "hidden";
        b.classList.remove("active");
        b.classList.remove("focus");
    } else if (c.style.visibility == "hidden" && !b.classList.contains("active")) {
        for (i = 1; i <= 7; i++) {
            if (document.getElementById("sidebarbutton" + i).classList.contains("active")) {
                document.getElementById("sidebarbutton" + i).classList.remove("active");
                document.getElementById("container" + i).style.visibility = "hidden";
            }
        }
        c.style.visibility = "visible";
        b.classList.add("active");
        b.classList.remove("focus");
    } else {
        c.style.visibility = "hidden";
        b.classList.remove("active");
        b.classList.remove("focus");
    }
}

function togglesetup() {
    document.getElementById("setup").classList.add("active");
    document.getElementById("setupbar").style.visibility = "visible";
    document.getElementById("scan").classList.remove("active");
    document.getElementById("scanbar").style.visibility = "hidden";

}

function togglescan() {
    document.getElementById("scan").classList.add("active");
    document.getElementById("scanbar").style.visibility = "visible";
    document.getElementById("setup").classList.remove("active");
    document.getElementById("setupbar").style.visibility = "hidden";
    for (i = 1; i <= 7; i++) {
        if (document.getElementById("sidebarbutton" + i).classList.contains("active")) {
            document.getElementById("sidebarbutton" + i).classList.remove("active");
            document.getElementById("container" + i).style.visibility = "hidden";
        }
    }
}