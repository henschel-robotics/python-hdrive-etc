function generateDebugValues(field, inputID) {
    var element = byId(field);
    element.innerHTML +=
    "<div class='liveBig'><h5>[index, subindex]</h5>" +
       "<input type='text' id='" + inputID + "'' size=5><input type='text' id='" + inputID +"_result' size=15>" +
        "<button >Set</button></div>";
}


// HTML field generator functions
function generateSlider(field, index, subindex, title, unit, text, min, max, divID = "") {
    let name = "m" + index + "s" + subindex;
    var element = byId(field);

    element.innerHTML +=
        "<div class='slidecontainer' id='" + divID + "'><h2>" + title + "</h2>" +
        "<div class='slidecontainer2'><input class='slider' type='range' id='" + name + "' for='" + name + "_I' min='" + min + "' max='" + max + "' oninput='" + name + "_I.value = this.value; sliderInput(this);' />" +
        "<div class='textBox'> " +
        "<input class='input_text' type='number' id='" + name + "_I' for='" + name + "' value=0 " +
        "onkeydown='if(event.key===\"Enter\"){submitSliderText(this);this.blur();}' " +
        "onchange='submitSliderText(this);' />" +
        "<h4>" + unit + "</h4></div></div></div>";
    fieldstoUpdate.push([index, subindex]);
}

function generateSelect(field, index, subindex, title, optionvalues, options, divName) {
    let name = "m" + index + "s" + subindex;
    var element = byId(field);
    var selectOptions = "";
    for (var i = 0; i < options.length; i++) 
        selectOptions += "<option value='" + optionvalues[i] + "'>" + options[i] + "</option>";
    element.innerHTML +=
        "<div class='box' id='" + divName + "'><h2>" + title + "</h2>" +
        "<div class='box_content'><div class='textBox'>" +
        "<select id=" + name + ">" + selectOptions + "</select></div>" +
        "<p>This is the option you can chose for the digital I/O Pin 1</p>" +
        "</div></div>";

    fieldstoUpdate.push([index, subindex]);
}

function addSaveToEEprom(field) {
    var element = byId(field);
    element.innerHTML +=
        "<div class='box'><h2>Save values and reset drive</h2>" +
        "<div class='box_content'>" +
        "<a href='#'' class='btn' onClick='saveToEEPROM();'>Save values to EEPROM</a>" +
        "<div class='saveEEPROM'></div></div></div>"
}

function generateBoxTitle(field, title, info, tag) {
    var element = byId(field);
    element.innerHTML +=
        "<div class='box wide'><"+tag+">" + title + "</"+tag+">"+
        "<h3>" + info + "</h3></div>";
}

function generateBoxContent(field, index, subindex, title, min, max, unit, desc, divToHide = "", inputType = "number", value = "") {
    let id = "m" + index + "s" + subindex;
    var element = byId(field);
    element.innerHTML +=
        "<div class='box' id=" + divToHide + "><h2>" + title + "</h2>" +
        "<div class='box_content'><div class='textBox'>" +
        "<input name='" + title + "' type='" + inputType + "' id=" + id + " max='" + max + "' min='" + min + "'" +
        (value !== "" ? " value='" + value + "'" : "") +
        " onInput='checkInputField(this);'  />" +
        "<h4>" + unit + "</h4>" +
        "</div><p>" + desc + "</p></div></div>";
    if (index !== "driveName")
        fieldstoUpdate.push([index, subindex]);
}

function generateBoxContentIP(field, index, subindex, title, desc) {
    let id = "m" + index + "s" + subindex;
    var element = byId(field);
    element.innerHTML +=
        "<div class='box'><h2>" + title + "</h2>" +
        "<div class='box_content'><div class='textBox wide'>" +
        "<input type='text' id=" + id + " name='inputIP' pattern='^(\\d{1,3}\\.){3}\\d{1,3}$' />" +
        "<h4></h4></div><p>" + desc + "</p></div></div>";
    fieldstoUpdateIP.push([index, subindex]);
}

function generateBoxContentMAC(field, index, subindex, title, desc) {
    let name = "m" + index + "s" + subindex;
    var element = byId(field);
    element.innerHTML +=
        "<div class='box'><h2>" + title + "</h2>" +
        "<div class='box_content'><div class='textBox wide'>" +
        "<input type='text' id=" + name + " disabled />" +
        "<h4></h4></div><p>" + desc + "</p></div></div>";

    fieldstoUpdateMAC.push([index, subindex]);
}

function generateSwitch(field, index, subindex, title, txt, divToToggle = "", data = "") {
    let name = "m" + index + "s" + subindex;
    var element = byId(field);
    element.innerHTML += "<div class='box'>" +
        "<h2>" + title + "</h2>" +
        "<div class='box_content'>" +
        "<div class='flipswitch'>" +
        "<input type='checkbox' name='flipswitch' class='flipswitch-cb' id='" + name + "'onchange='checkboxChangeHandler(this, \"" + divToToggle + "\", " + data + ");' >" +
        "<label class='flipswitch-label' for='" + name + "'>" +
        "<div class='flipswitch-inner'></div>" +
        "<div class='flipswitch-switch'></div>" +
        "</label></div><p>" + txt + "</p></div></div>";

    fieldstoUpdateSwitch.push([index, subindex, divToToggle, data]);
}

function generateButton(field, name, btnTitle, title, txt, color = "") {
    var element = byId(field);
    element.innerHTML += "<div class='box'>" +
        "<h2>" + title + "</h2><div class='box_content'>" +
        "<a href='#'' class='btn " + color + "'>" + btnTitle + "</a><br />" +
        "<p>" + txt + "</p></div></div>";
}

function generateLiveContent(field, index, subindex, title, unit) {
    var element = byId(field);
    element.innerHTML +=
        "<div class='live-row'>" +
        "<h2>" + title + "</h2>" +
        "<h3><div id='m" + index + "s" + subindex + "'>0</div></h3>" +
        "<h4>" + unit + "</h4>" +
        "</div>";
}

function generateLiveContentTitle(field, title) {
    var element = byId(field);
    element.innerHTML += "<h1>" + title + "</h1>";
}
