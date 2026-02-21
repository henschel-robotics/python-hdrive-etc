// gets the element by id
var ROM_byId = function (id) { return document.getElementById(id); };

// gets the element by id
var byId = function (id) { return document.getElementById(id); };

// Set value of a element
function ROM_setV(name, v) {
    if (v != null && name != null) {
        if (byId(name) != null)
            byId(name).value = v;
    }
}

async function ROM_readName() {
    const url = `${host}getData.cgi?nam`;

    try {
        const response = await fetch(url);

        if (!response.ok) {
            throw new Error(`Failed to fetch data (${response.status})`);
            throw new Error(`Failed to fetch data (${response.status})`);
        }

        const text = await response.text();
        return text;
    } catch (error) {
        console.error(error);
        return null;
    }
}

async function ROM_readOBJ(index, subindex) {
    if (!index && index !== 0) {
        console.error("ROM_readOBJ::index is empty");
        return;
    }

    const url = `${host}getData.cgi?obj=${index}&sub=${subindex}`;

    try {
        const response = await fetch(url);

        if (!response.ok) {
            throw new Error(`Failed to fetch data (${response.status})`);
        }

        const text = await response.text();
        return text;
    } catch (error) {
        console.error(error);
        return null;
    }
}

async function ROM_writeTicket(data) {
    if (isNaN(index)) {
        console.error("ROM_writeOBJ::index is zero");
        return;
    }

    const url = `${host}writeTicket`;
    const formData = new FormData();
    const xmlString = `<system a="${data}" d="" cd="" d="" />`;

    formData.append("xml", xmlString);

    try {
        const response = await fetch(url, {
            method: "POST",
            body: formData
        });

        if (!response.ok) {
            throw new Error(`Failed to write object (${response.status})`);
        }

        return true;
    } catch (error) {
        console.error(error);
        return false;
    }
}

async function ROM_sendXHRRequest(post, host, blob = null, requestHeader = null) {
    const xhr = new XMLHttpRequest();
    xhr.open(post, host, true);
    if (requestHeader != null)
        xhr.setRequestHeader("Content-Type", requestHeader);

    return new Promise((resolve, reject) => {
        xhr.onloadend = () => {
            if (xhr.status >= 200 && xhr.status < 300) {
                resolve(xhr.response);
            } else {
                reject(new Error(`XHR request failed with status ${xhr.status}`));
            }
        };

        xhr.onerror = () => {
            reject(new Error("XHR request encountered an error"));
        };

        xhr.send(blob);
    });
}

async function ROM_writeOBJ(index, subindex, value) {
    const string = `<objWrite a="${index}" b="${subindex}" c="${value}" />`;
    const blob = new Blob([string], { type: "text/plain" });

    try {
        await ROM_sendXHRRequest("POST", `${host}writeTicket`, blob);
    } catch (error) {
        console.error(error);
    }
}

async function ROM_writeOBJList(objectList) {
    for (let ii = 0; ii < objectList.length; ++ii) {
        let [index, subindex, value] = objectList[ii];

        const string = `<objWrite a="${index}" b="${subindex}" c="${value}" />`;
        const blob = new Blob([string], { type: "text/plain" });

        try {
            await ROM_sendXHRRequest("POST", `${host}writeTicket`, blob);
        } catch (error) {
            console.error(error);
        }
    }
}

// Look up a value from the objectsFromDrive map (keys are hex strings like "6064,0")
function getObjValue(index, subindex) {
    let key = index + "," + subindex;
    return objectsFromDrive.has(key) ? objectsFromDrive.get(key) : undefined;
}

async function ROM_readOBJList(callBack) {
    const url = `${host}getData.cgi?objL`;

    try {
        const response = await fetch(url);
        if (!response.ok) throw new Error(`Failed to fetch data (${response.status})`);

        const text = await response.text();
        // Format from server: "decimal_index,decimal_subindex,value;..."
        // Convert to hex string keys for lookup
        const entries = text.split(";");
        entries.forEach(entry => {
            const parts = entry.split(",");
            if (parts.length >= 3) {
                let idx = parseInt(parts[0]).toString(16);  // decimal → hex string
                let sub = parseInt(parts[1]).toString(16);
                let raw = parts.slice(2).join(",");
                let num = parseFloat(raw);
                let val = (isNaN(num) || String(num) !== raw.trim()) ? raw : num;
                objectsFromDrive.set(idx + "," + sub, val);
            }
        });

        callBack();
        return text;
    } catch (error) {
        console.error(error);
        return null;
    }
}