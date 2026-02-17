#include "webpage.hpp"

#include <FreeRTOS.h>
#include <lwip/api.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "heaters.hpp"
#include "puller.hpp"
#include "spooler.hpp"

// Global static buffer - avoids stack issues. Increased size slightly for
// safety.
static char g_resp_buf[3072];

// ============================================================================
//  FRONTEND (HTML)
// ============================================================================
const char *INDEX_HTML = R"HTML(
HTTP/1.1 200 OK
Content-Type: text/html
Connection: close

<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Extruder Control</title>
    <style>
        :root { --primary: #0d6efd; --bg: #f8f9fa; --card: #ffffff; }
        body { font-family: sans-serif; background: var(--bg); padding: 20px; }
        .dashboard { display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 20px; }
        .card { background: var(--card); border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); padding: 20px; }
        .btn { width: 100%; padding: 10px; cursor: pointer; border: none; border-radius: 4px; font-weight: bold; margin-top: 5px; color: white; }
        .btn-sm { padding: 4px 8px; font-size: 0.9rem; cursor: pointer; }
        .status { padding: 2px 8px; border-radius: 10px; font-size: 0.8rem; font-weight: bold; }
        .on { background: #d1e7dd; color: #0f5132; }
        .off { background: #f8d7da; color: #842029; }
        .tune { background: #fff3cd; color: #664d03; }
        
        .control-row { display: flex; align-items: center; gap: 5px; margin-top: 8px; }
        .pid-inp { width: 40px; padding: 4px; }
        hr { border: 0; border-top: 1px solid #eee; margin: 15px 0; }
        label { font-size: 0.8rem; font-weight: bold; color: #555; }
    </style>
</head>
<body>
    <h1>Extruder Dashboard</h1>
    
    <div class="dashboard">
        <div class="card">
            <h3>Puller <span id="p-st" class="status off">OFF</span></h3>
            <p>Dia: <b id="p-dia">--</b> mm (Filt: <b id="p-fdia">--</b>)</p>
            <p>Vel: <b id="p-vel">--</b> mm/s | Flow: <b id="p-flow">--</b></p>
            <button id="p-btn" class="btn" style="background:#198754" onclick="cmd('puller')">TOGGLE PULLER</button>
        </div>
        <div class="card">
            <h3>Spooler <span id="s-st" class="status off">OFF</span></h3>
            <p>Pos: <b id="s-pos">--</b> mm</p>
            <button id="s-btn" class="btn" style="background:#198754" onclick="cmd('spooler')">TOGGLE SPOOLER</button>
        </div>
    </div>

    <h3>Heaters <span id="h-st" class="status off">OFF</span></h3>
    <button id="h-btn" class="btn" style="background:#198754; width:200px" onclick="cmd('heaters')">TOGGLE ALL</button>
    <div class="dashboard" id="zones" style="margin-top:20px"></div>

    <script>
        // Generate Zone Cards
        for(let i=0; i<4; i++) {
            document.getElementById('zones').innerHTML += `
            <div class="card">
                <h4>Zone ${i+1} <span id="tunest${i}" class="status tune" style="display:none">TUNING</span></h4>
                <div>Curr: <b id="t${i}" style="font-size:1.2em">--</b>&deg;C</div>
                <div>Targ: <b id="tg${i}">--</b> | Duty: <b id="dc${i}">--</b>%</div>
                
                <div class="control-row">
                   <input type="number" id="in${i}" style="width:60px" placeholder="Temp"> 
                   <button class="btn-sm" onclick="setT(${i})">Set Temp</button>
                </div>

                <hr>
                
                <div class="control-row">
                    <label>P</label><input type="number" step="0.1" class="pid-inp" id="kp${i}">
                    <label>I</label><input type="number" step="0.01" class="pid-inp" id="ki${i}">
                    <label>D</label><input type="number" step="0.1" class="pid-inp" id="kd${i}">
                    <button class="btn-sm" onclick="setPID(${i})">Upd PID</button>
                </div>
                
                <div class="control-row" style="margin-top:10px">
                    <label>Tune @</label>
                    <input type="number" id="tuneval${i}" style="width:50px" value="200">
                    <button class="btn-sm" style="background:#ffc107" onclick="runTune(${i})">Autotune</button>
                </div>
            </div>`;
        }

        async function update() {
            try {
                const res = await fetch('/api/data?t=' + Date.now());
                const d = await res.json();
                
                // Global Status
                const hOn = d.heaters.running;
                document.getElementById('h-st').innerText = hOn ? "ON" : "OFF";
                document.getElementById('h-st').className = "status " + (hOn?"on":"off");

                // Zones
                d.heaters.zones.forEach((z, i) => {
                    document.getElementById(`t${i}`).innerText = z.curr.toFixed(1);
                    document.getElementById(`tg${i}`).innerText = z.targ.toFixed(0);
                    document.getElementById(`dc${i}`).innerText = (z.duty*100).toFixed(0);

                    // Tuning Status
                    const ts = document.getElementById(`tunest${i}`);
                    ts.style.display = z.tuning ? "inline-block" : "none";

                    // Update PID inputs ONLY if the user isn't currently typing in them
                    const act = document.activeElement ? document.activeElement.id : "";
                    if(act !== `kp${i}`) document.getElementById(`kp${i}`).value = z.kp;
                    if(act !== `ki${i}`) document.getElementById(`ki${i}`).value = z.ki;
                    if(act !== `kd${i}`) document.getElementById(`kd${i}`).value = z.kd;
                });

                // Puller & Spooler
                document.getElementById('p-st').innerText = d.puller.running ? "ON" : "OFF";
                document.getElementById('p-st').className = "status " + (d.puller.running?"on":"off");
                document.getElementById('p-dia').innerText = d.puller.dia.toFixed(3);
                document.getElementById('p-fdia').innerText = d.puller.fdia.toFixed(3);
                document.getElementById('p-vel').innerText = d.puller.vel.toFixed(2);
                document.getElementById('p-flow').innerText = d.puller.flow.toFixed(1);

                document.getElementById('s-st').innerText = d.spooler.running ? "ON" : "OFF";
                document.getElementById('s-st').className = "status " + (d.spooler.running?"on":"off");
                document.getElementById('s-pos').innerText = d.spooler.pos.toFixed(1);
            } catch(e) {}
        }

        function cmd(type) {
            const isRun = document.getElementById(type[0]+'-st').innerText === "ON";
            fetch(`/api/ctrl?type=${type}&cmd=${isRun?'stop':'start'}`, {method:'POST'}).then(() => setTimeout(update, 200));
        }
        function setT(z) {
            const v = document.getElementById('in'+z).value;
            if(v) fetch(`/api/set?z=${z}&v=${v}`, {method:'POST'}).then(() => setTimeout(update, 200));
        }
        function setPID(z) {
            const p = document.getElementById('kp'+z).value;
            const i = document.getElementById('ki'+z).value;
            const d = document.getElementById('kd'+z).value;
            fetch(`/api/pid?z=${z}&p=${p}&i=${i}&d=${d}`, {method:'POST'}).then(() => setTimeout(update, 200));
        }
        function runTune(z) {
            const v = document.getElementById('tuneval'+z).value;
            if(confirm('Start Autotune for Zone ' + (z+1) + '?')) {
                fetch(`/api/tune?z=${z}&v=${v}`, {method:'POST'}).then(() => setTimeout(update, 200));
            }
        }
        setInterval(update, 1000);
        update();
    </script>
</body>
</html>
)HTML";

// ============================================================================
//  BACKEND HELPERS
// ============================================================================

void respond_ok(struct netconn *conn) {
    const char *resp =
        "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: "
        "close\r\n\r\nOK";
    netconn_write(conn, resp, strlen(resp), NETCONN_NOCOPY);
}

// Helper to get all data for a zone to reduce code repetition
struct ZoneData {
    float curr, targ, duty, kp, ki, kd;
    bool tuning;
};

ZoneData get_zone_data(heaters::Zone z) {
    ZoneData d;
    d.curr = heaters::get_temp(z);
    d.targ = heaters::get_target_temp(z);
    d.duty = heaters::get_duty_cycle(z);
    heaters::get_pid(z, &d.kp, &d.ki, &d.kd);
    d.tuning = heaters::is_tuning(z);
    return d;
}

// ============================================================================
//  API HANDLERS
// ============================================================================

void handle_api_data(struct netconn *conn) {
    ZoneData z0 = get_zone_data(heaters::Zone::Z1);
    ZoneData z1 = get_zone_data(heaters::Zone::Z2);
    ZoneData z2 = get_zone_data(heaters::Zone::Z3);
    ZoneData z3 = get_zone_data(heaters::Zone::Z4);

    // Format JSON with PID and Tuning data
    snprintf(g_resp_buf, sizeof(g_resp_buf),
             "{\"heaters\":{\"running\":%s,\"zones\":["
             // Zone 1
             "{\"curr\":%.1f,\"targ\":%.1f,\"duty\":%.2f,\"kp\":%.2f,\"ki\":%."
             "3f,\"kd\":%.2f,\"tuning\":%s},"
             // Zone 2
             "{\"curr\":%.1f,\"targ\":%.1f,\"duty\":%.2f,\"kp\":%.2f,\"ki\":%."
             "3f,\"kd\":%.2f,\"tuning\":%s},"
             // Zone 3
             "{\"curr\":%.1f,\"targ\":%.1f,\"duty\":%.2f,\"kp\":%.2f,\"ki\":%."
             "3f,\"kd\":%.2f,\"tuning\":%s},"
             // Zone 4
             "{\"curr\":%.1f,\"targ\":%.1f,\"duty\":%.2f,\"kp\":%.2f,\"ki\":%."
             "3f,\"kd\":%.2f,\"tuning\":%s}"
             "]},\"puller\":{\"running\":%s,\"dia\":%.3f,\"fdia\":%.3f,\"vel\":"
             "%.2f,\"flow\":%.2f},"
             "\"spooler\":{\"running\":%s,\"pos\":%.1f}}",

             heaters::is_running() ? "true" : "false",
             // Z1
             z0.curr, z0.targ, z0.duty, z0.kp, z0.ki, z0.kd,
             z0.tuning ? "true" : "false",
             // Z2
             z1.curr, z1.targ, z1.duty, z1.kp, z1.ki, z1.kd,
             z1.tuning ? "true" : "false",
             // Z3
             z2.curr, z2.targ, z2.duty, z2.kp, z2.ki, z2.kd,
             z2.tuning ? "true" : "false",
             // Z4
             z3.curr, z3.targ, z3.duty, z3.kp, z3.ki, z3.kd,
             z3.tuning ? "true" : "false",

             puller::is_running() ? "true" : "false", puller::get_diameter(),
             puller::get_filtered_diameter(), puller::get_velocity(),
             puller::get_flow_rate(), spooler::is_running() ? "true" : "false",
             spooler::get_position());

    const char *hdr =
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: "
        "close\r\n\r\n";
    netconn_write(conn, hdr, strlen(hdr), NETCONN_COPY);
    netconn_write(conn, g_resp_buf, strlen(g_resp_buf), NETCONN_COPY);
}

void handle_api_ctrl(struct netconn *conn, char *req) {
    if (strstr(req, "type=heaters")) {
        strstr(req, "cmd=start") ? heaters::start() : heaters::stop();
    } else if (strstr(req, "type=puller")) {
        strstr(req, "cmd=start") ? puller::start() : puller::stop();
    } else if (strstr(req, "type=spooler")) {
        strstr(req, "cmd=start") ? spooler::start() : spooler::stop();
    }
    respond_ok(conn);
}

void handle_api_set(struct netconn *conn, char *req) {
    char *z_ptr = strstr(req, "z=");
    char *v_ptr = strstr(req, "v=");
    if (z_ptr && v_ptr) {
        int z = atoi(z_ptr + 2);
        float v = (float)strtod(v_ptr + 2, NULL);
        if (z >= 0 && z < 4) heaters::set_temp((heaters::Zone)z, v);
    }
    respond_ok(conn);
}

void handle_api_pid(struct netconn *conn, char *req) {
    // Expected format: ?z=0&p=10.5&i=0.2&d=50
    char *z_ptr = strstr(req, "z=");
    char *p_ptr = strstr(req, "p=");
    char *i_ptr = strstr(req, "i=");
    char *d_ptr = strstr(req, "d=");

    if (z_ptr && p_ptr && i_ptr && d_ptr) {
        int z = atoi(z_ptr + 2);
        float p = (float)strtod(p_ptr + 2, NULL);
        float i = (float)strtod(i_ptr + 2, NULL);
        float d = (float)strtod(d_ptr + 2, NULL);
        if (z >= 0 && z < 4) {
            heaters::set_pid((heaters::Zone)z, p, i, d);
        }
    }
    respond_ok(conn);
}

void handle_api_tune(struct netconn *conn, char *req) {
    // Expected format: ?z=0&v=200
    char *z_ptr = strstr(req, "z=");
    char *v_ptr = strstr(req, "v=");
    if (z_ptr && v_ptr) {
        int z = atoi(z_ptr + 2);
        float temp = (float)strtod(v_ptr + 2, NULL);
        if (z >= 0 && z < 4) {
            heaters::start_autotune((heaters::Zone)z, temp);
        }
    }
    respond_ok(conn);
}

// ============================================================================
//  MAIN TASK
// ============================================================================

void http_server_task(void *arg) {
    (void)arg;
    struct netconn *conn, *newconn;
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 80);
    netconn_listen(conn);

    while (1) {
        if (netconn_accept(conn, &newconn) == ERR_OK) {
            struct netbuf *buf;
            void *data;
            u16_t len;
            if (netconn_recv(newconn, &buf) == ERR_OK) {
                netbuf_data(buf, &data, &len);
                // Ensure null termination for string ops
                char req[512];
                u16_t copy_len = (len > 511) ? 511 : len;
                memcpy(req, data, copy_len);
                req[copy_len] = '\0';

                // Simple Routing
                if (strncmp(req, "GET /api/data", 13) == 0)
                    handle_api_data(newconn);
                else if (strncmp(req, "POST /api/ctrl", 14) == 0)
                    handle_api_ctrl(newconn, req);
                else if (strncmp(req, "POST /api/set", 13) == 0)
                    handle_api_set(newconn, req);
                else if (strncmp(req, "POST /api/pid", 13) == 0)
                    handle_api_pid(newconn, req);
                else if (strncmp(req, "POST /api/tune", 14) == 0)
                    handle_api_tune(newconn, req);
                else if (strncmp(req, "GET / ", 6) == 0 ||
                         strncmp(req, "GET /index", 10) == 0) {
                    netconn_write(newconn, INDEX_HTML, strlen(INDEX_HTML),
                                  NETCONN_NOCOPY);
                }
                netbuf_delete(buf);
            }
            netconn_close(newconn);
            netconn_delete(newconn);
        }
    }
}
