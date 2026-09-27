// Panel ramienia SO-101: wspolny dla arm_panel.html (serwer tools/arm_web.py, :8010)
// i frontend.html (panel jazdy web_control.py, :8000 - gada z :8010 przez CORS).
//
//   const arm = mountArmPanel(document.getElementById("arm-root"), {
//     base: "http://" + location.hostname + ":8010",  // "" = ten sam serwer
//     stickyStop: false,    // duzy STOP przyklejony do dolu ekranu
//     keyboardStop: false,  // spacja = STOP ramienia
//   });
//   arm.stop();             // np. z glownego przycisku STOP jazdy
//   arm.motions();          // nazwy ruchow z motions/ (ostatni stan serwera)
//
// Sekcja "NAGRYWANIE RUCHU": biezaca poza -> punkt szkicu (add_point), szkic -> motions/<nazwa>.json
// (save_motion). Szkic trzyma serwer (pinecone_bot/arm_panel.py), wiec przezywa odswiezenie strony.
//
// Style sa pod klasa .armp, zeby nie gryzly sie z frontend.html.
// DOM budowany raz; przy odpytywaniu stanu zmieniany jest tylko tekst i pozycje
// znacznikow (docs/HARDWARE.md, pulapka 32).
(function () {
  const CSS = `
  .armp { --armp-ok: #35c759; --armp-bad: #ff453a; --armp-warn: #ffb340; --armp-accent: #4f9dff;
          --armp-muted: #8a8f98; --armp-text: #e6e8eb; --armp-btn: #22262f; --armp-border: #2c313c; }
  .armp .armp-sec { margin-bottom: 14px; }
  .armp .armp-title { font-size: 12px; font-weight: 700; letter-spacing: 0.04em; color: var(--armp-muted); margin-bottom: 8px; }
  .armp .armp-status { display: flex; align-items: center; gap: 8px; font-size: 14px; }
  .armp .armp-dot { width: 10px; height: 10px; border-radius: 50%; background: var(--armp-bad); flex-shrink: 0; }
  .armp .armp-dot.on { background: var(--armp-ok); }
  .armp .armp-dot.busy { background: var(--armp-warn); }
  .armp .armp-muted { color: var(--armp-muted); font-size: 12px; }
  .armp .armp-error { color: var(--armp-bad); font-size: 12px; margin-top: 4px; min-height: 1em; word-break: break-word; }
  .armp button {
    font: inherit; color: var(--armp-text); background: var(--armp-btn);
    border: 1px solid var(--armp-border); border-radius: 8px; cursor: pointer;
    user-select: none; -webkit-user-select: none; touch-action: manipulation;
  }
  .armp button:active { background: var(--armp-accent); border-color: var(--armp-accent); color: #fff; }
  .armp button:disabled { opacity: 0.35; cursor: default; }
  .armp .armp-steps { display: flex; gap: 8px; }
  .armp .armp-steps button { flex: 1; padding: 10px; font-weight: 600; }
  .armp .armp-steps button.sel { background: var(--armp-accent); border-color: var(--armp-accent); color: #fff; }
  .armp .armp-joint { display: grid; grid-template-columns: 52px 1fr 52px; gap: 8px; align-items: center; margin-bottom: 10px; }
  .armp .armp-joint button { height: 52px; font-size: 24px; font-weight: 700; }
  .armp .armp-jinfo { min-width: 0; }
  .armp .armp-jname { font-size: 13px; display: flex; justify-content: space-between; gap: 6px; }
  .armp .armp-jval { font-variant-numeric: tabular-nums; font-weight: 600; }
  .armp .armp-track { height: 8px; background: var(--armp-btn); border-radius: 4px; position: relative; margin: 6px 0 3px; }
  .armp .armp-mark { position: absolute; top: -3px; width: 3px; height: 14px; border-radius: 2px; transform: translateX(-1px); }
  .armp .armp-mark.pos { background: var(--armp-ok); }
  .armp .armp-mark.set { background: var(--armp-accent); opacity: 0.7; }
  .armp .armp-jlim { font-size: 11px; color: var(--armp-muted); display: flex; justify-content: space-between; font-variant-numeric: tabular-nums; }
  .armp .armp-row { display: flex; gap: 8px; flex-wrap: wrap; }
  .armp .armp-row button { flex: 1 1 45%; padding: 12px; font-weight: 600; min-width: 100px; }
  .armp .armp-stop { width: 100%; padding: 16px; font-size: 18px; font-weight: 800;
                     background: var(--armp-bad); border-color: var(--armp-bad); color: #fff; }
  .armp .armp-stop:active { background: #c7322a; border-color: #c7322a; }
  .armp .armp-stopbar { position: fixed; left: 0; right: 0; bottom: 0; padding: 12px 16px;
                        background: linear-gradient(transparent, #0f1115 30%); display: flex; justify-content: center; z-index: 10; }
  .armp .armp-stopbar .armp-stop { max-width: 520px; font-size: 20px; padding: 18px; }
  .armp .armp-form { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; margin-bottom: 8px; }
  .armp .armp-form input[type=text], .armp .armp-form input[type=number] {
    font: inherit; color: var(--armp-text); background: var(--armp-btn); border: 1px solid var(--armp-border);
    border-radius: 8px; padding: 8px; min-width: 0; }
  .armp .armp-form input[type=text] { flex: 1 1 140px; }
  .armp .armp-form input[type=number] { width: 72px; }
  .armp .armp-chk { font-size: 12px; color: var(--armp-muted); display: flex; align-items: center; gap: 4px; white-space: nowrap; }
  .armp .armp-draft { font-size: 12px; margin: 6px 0 10px; font-variant-numeric: tabular-nums; }
  .armp .armp-draft div { padding: 2px 0; border-bottom: 1px solid var(--armp-border); }
  .armp .armp-warn { color: var(--armp-warn); font-size: 12px; margin-bottom: 8px; }
  .armp .armp-xyzgrid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; margin-top: 8px; }
  .armp .armp-xyzgrid button { height: 52px; font-weight: 700; }
  .armp .armp-tcp { font-variant-numeric: tabular-nums; font-size: 14px; margin-bottom: 8px; }
  `;

  const OFFLINE = "Serwer ramienia nie działa (na Pi: python tools/arm_web.py)";

  function mountArmPanel(root, opts) {
    opts = opts || {};
    const base = opts.base || "";

    if (!document.getElementById("armp-style")) {
      const st = document.createElement("style");
      st.id = "armp-style";
      st.textContent = CSS;
      document.head.appendChild(st);
    }

    root.classList.add("armp");
    root.innerHTML = `
      <div class="armp-sec">
        <div class="armp-status"><span class="armp-dot"></span><span class="armp-st">łączę z serwerem ramienia...</span></div>
        <div class="armp-muted armp-result"></div>
        <div class="armp-error"></div>
      </div>
      <div class="armp-sec">
        <div class="armp-title">KROK (stopnie, chwytak: jednostki 0-100)</div>
        <div class="armp-steps"></div>
      </div>
      <div class="armp-sec">
        <div class="armp-title">STAWY <span class="armp-muted">(zielony = odczyt z serw, niebieski = ostatnia komenda)</span></div>
        <div class="armp-joints"></div>
      </div>
      <div class="armp-sec armp-xyz" style="display:none">
        <div class="armp-title">JOG XYZ <span class="armp-muted">(końcówka chwytaka, mm; pochylenie chwytaka bez zmian)</span></div>
        <div class="armp-tcp">TCP: -</div>
        <div class="armp-steps armp-xyzsteps"></div>
        <div class="armp-xyzgrid">
          <button data-ax="z" data-d="1">GÓRA</button>
          <button data-ax="x" data-d="1">PRZÓD</button>
          <button data-ax="y" data-d="1">LEWO</button>
          <button data-ax="z" data-d="-1">DÓŁ</button>
          <button data-ax="x" data-d="-1">TYŁ</button>
          <button data-ax="y" data-d="-1">PRAWO</button>
        </div>
        <div class="armp-row" style="margin-top:8px">
          <button class="armp-zero" data-always="1">ZERO URDF (ramię prosto, poziomo do przodu)</button>
        </div>
        <div class="armp-muted">Raz po kalibracji: ustaw stawami ramię wyprostowane poziomo do przodu, chwytak w linii, i kliknij ZERO URDF. Jeśli GÓRA jedzie w dół albo w bok, zmień znak stawu w pinecone_config.json (arm.urdf_sign).</div>
      </div>
      <div class="armp-sec armp-poses">
        <div class="armp-title">POZYCJE</div>
        <div class="armp-row">
          <button data-cmd="home" data-always="1">HOME</button>
          <button data-cmd="open">Otwórz chwytak</button>
          <button data-cmd="close">Zamknij chwytak</button>
        </div>
      </div>
      <div class="armp-sec armp-mot">
        <div class="armp-title">RUCHY Z motions/</div>
        <div class="armp-warn armp-mot-warn" style="display:none">tryb bez HOME: odtwarzaj tylko ruchy nagrane pod aktualny montaz (grasp_mid/home/drop_box koncza w HOME i uderza w kamere)</div>
        <div class="armp-row armp-motions"></div>
      </div>
      <div class="armp-sec armp-rec">
        <div class="armp-title">NAGRYWANIE RUCHU <span class="armp-muted">(ustaw stawami, dodaj punkt, zapisz)</span></div>
        <div class="armp-form">
          <input type="text" class="armp-label" placeholder="etykieta punktu (np. nad szyszka)" maxlength="40" />
          <input type="number" class="armp-secs" min="0" max="30" step="0.1" value="1.5" title="czas dojazdu do punktu [s]" />
          <label class="armp-chk"><input type="checkbox" class="armp-check" /> zacisk (check_gripper)</label>
        </div>
        <div class="armp-row">
          <button class="armp-add" data-always="1">+ PUNKT (biezaca poza)</button>
          <button class="armp-drop" data-always="1">usun ostatni</button>
        </div>
        <div class="armp-draft armp-muted"></div>
        <div class="armp-form">
          <input type="text" class="armp-mname" placeholder="nazwa ruchu (np. grasp_cam)" maxlength="40" />
          <label class="armp-chk"><input type="checkbox" class="armp-over" /> nadpisz istniejacy</label>
        </div>
        <div class="armp-row">
          <button class="armp-save" data-always="1">ZAPISZ do motions/</button>
          <button class="armp-clear" data-always="1">wyczysc szkic</button>
        </div>
        <div class="armp-muted">Chwytak zapisuje sie z ostatniej komendy: przed punktem "zacisk" kliknij "Zamknij chwytak". Sekundy = czas dojazdu do punktu.</div>
      </div>
      <div class="${opts.stickyStop ? "armp-stopbar" : "armp-sec"}">
        <button class="armp-stop" data-always="1">STOP RAMIENIA${opts.keyboardStop ? " (spacja)" : ""}</button>
      </div>`;

    const q = (sel) => root.querySelector(sel);
    const el = {
      dot: q(".armp-dot"), st: q(".armp-st"), result: q(".armp-result"), error: q(".armp-error"),
      steps: q(".armp-steps"), joints: q(".armp-joints"), motions: q(".armp-motions"),
      draft: q(".armp-draft"), label: q(".armp-label"), secs: q(".armp-secs"), check: q(".armp-check"),
      mname: q(".armp-mname"), over: q(".armp-over"), motWarn: q(".armp-mot-warn"),
      xyz: q(".armp-xyz"), xyzSteps: q(".armp-xyzsteps"), tcp: q(".armp-tcp"),
    };
    const marks = {};  // joint -> {v, p, s, lo, hi}
    let step = 5;
    let xyzStep = 10;
    let xyzBuilt = false;
    let built = false;
    let motionsKey = "";
    let draftKey = "";
    let lastMotions = [];
    let sendError = "";

    function showError(msg) {
      sendError = msg;
      el.error.textContent = msg;
    }

    async function send(cmd, showMsg) {
      try {
        const r = await fetch(base + "/api/cmd", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(cmd),
        });
        const j = await r.json();
        if (!j.ok) showError("Odrzucone: " + j.msg);
        else if (sendError) showError("");
        if (j.ok && showMsg) el.result.textContent = j.msg;
        return j.ok;
      } catch (e) {
        showError(OFFLINE);
        return false;
      }
    }

    function buildSteps(steps) {
      el.steps.textContent = "";
      for (const s of steps) {
        const b = document.createElement("button");
        b.textContent = s;
        b.dataset.step = s;
        b.dataset.always = "1";
        if (s === step) b.classList.add("sel");
        b.onclick = () => {
          step = s;
          for (const x of el.steps.children) x.classList.toggle("sel", Number(x.dataset.step) === step);
        };
        el.steps.appendChild(b);
      }
    }

    function buildXyz(steps) {
      el.xyz.style.display = "";
      el.xyzSteps.textContent = "";
      for (const s of steps) {
        const b = document.createElement("button");
        b.textContent = s + " mm";
        b.dataset.step = s;
        b.dataset.always = "1";
        if (s === xyzStep) b.classList.add("sel");
        b.onclick = () => {
          xyzStep = s;
          for (const x of el.xyzSteps.children) x.classList.toggle("sel", Number(x.dataset.step) === xyzStep);
        };
        el.xyzSteps.appendChild(b);
      }
      el.xyz.querySelectorAll("[data-ax]").forEach((b) => {
        b.onclick = () => send({ cmd: "jog_xyz", axis: b.dataset.ax, step_mm: Number(b.dataset.d) * xyzStep });
      });
    }

    function buildJoints(joints) {
      el.joints.textContent = "";
      for (const j of joints) {
        const row = document.createElement("div");
        row.className = "armp-joint";
        row.innerHTML =
          `<button data-j="${j}" data-d="-1">−</button>` +
          `<div class="armp-jinfo">` +
            `<div class="armp-jname"><span>${j}</span><span class="armp-jval">-</span></div>` +
            `<div class="armp-track"><span class="armp-mark set"></span><span class="armp-mark pos"></span></div>` +
            `<div class="armp-jlim"><span></span><span></span></div>` +
          `</div>` +
          `<button data-j="${j}" data-d="1">+</button>`;
        const lim = row.querySelectorAll(".armp-jlim span");
        marks[j] = {
          v: row.querySelector(".armp-jval"), p: row.querySelector(".armp-mark.pos"),
          s: row.querySelector(".armp-mark.set"), lo: lim[0], hi: lim[1],
        };
        row.querySelectorAll("button").forEach((b) => {
          b.onclick = () => send({ cmd: "jog", joint: b.dataset.j, step: Number(b.dataset.d) * step });
        });
        el.joints.appendChild(row);
      }
    }

    function buildMotions(names) {
      el.motions.textContent = "";
      for (const n of names) {
        const b = document.createElement("button");
        b.textContent = n;
        b.onclick = () => send({ cmd: "motion", name: n });
        el.motions.appendChild(b);
      }
      if (!names.length) el.motions.innerHTML = '<span class="armp-muted">brak plików w motions/</span>';
    }

    function buildDraft(draft) {
      if (!draft.length) { el.draft.innerHTML = '<span class="armp-muted">szkic pusty</span>'; return; }
      let total = 0;
      el.draft.innerHTML = draft.map((wp, i) => {
        total += wp.seconds;
        const pose = Object.entries(wp.pose).map(([j, v]) => `${j.replace("shoulder_", "sh_").replace("wrist_", "wr_").replace("elbow_flex", "elbow")}=${v}`).join(" ");
        return `<div>${i + 1}. <b>${wp.label}</b> ${wp.seconds}s${wp.check_gripper ? " [zacisk]" : ""} <span class="armp-muted">${pose}</span></div>`;
      }).join("") + `<div>razem ${draft.length} pkt, ${total.toFixed(1)} s</div>`;
    }

    function placeMark(m, v, lo, hi) {
      if (v === undefined || v === null) { m.style.display = "none"; return; }
      m.style.display = "";
      const f = Math.max(0, Math.min(1, (v - lo) / (hi - lo)));
      m.style.left = (f * 100).toFixed(1) + "%";
    }

    function busyText(b) {
      if (b.cmd === "jog") return `ruch: ${b.joint} ${b.step > 0 ? "+" : ""}${b.step}`;
      if (b.cmd === "motion") return `ruch: ${b.name}`;
      if (b.cmd === "jog_xyz") return `ruch: TCP ${b.axis} ${b.step_mm > 0 ? "+" : ""}${b.step_mm} mm`;
      return "ruch: " + b.cmd;
    }

    function setEnabled(homed) {
      root.querySelectorAll("button").forEach((b) => {
        if (!b.dataset.always) b.disabled = !homed;
      });
    }

    function render(s) {
      if (!built) {
        buildSteps(s.steps);
        buildJoints(s.joints);
        built = true;
      }
      if (!xyzBuilt && s.xyz_steps && s.xyz_steps.length) { buildXyz(s.xyz_steps); xyzBuilt = true; }
      if (s.tcp) el.tcp.textContent = `TCP: x ${s.tcp.x} y ${s.tcp.y} z ${s.tcp.z} mm, pochylenie ${s.tcp.pitch}°`;
      const key = s.motions.join(",");
      if (key !== motionsKey) { buildMotions(s.motions); motionsKey = key; lastMotions = s.motions.slice(); }
      const dkey = JSON.stringify(s.draft || []);
      if (dkey !== draftKey) { buildDraft(s.draft || []); draftKey = dkey; }

      el.dot.className = "armp-dot " + (s.busy ? "busy" : "on");
      let st = s.busy ? busyText(s.busy) : "gotowe";
      if (s.queue) st += ` (w kolejce: ${s.queue})`;
      if (!s.homed) st = (s.busy ? "jadę do HOME..." : "czekam na HOME") + " - inne komendy zablokowane";
      if (s.manual_only) st += " | tryb bez HOME";
      // --no-home: HOME wylaczone na serwerze, chowamy je; ruchy z motions/ zostaja z ostrzezeniem
      el.motWarn.style.display = s.manual_only ? "" : "none";
      q('[data-cmd="home"]').style.display = s.manual_only ? "none" : "";
      el.st.textContent = st;
      el.result.textContent = s.result || "";
      if (s.error) el.error.textContent = "Błąd: " + s.error;
      else if (!sendError) el.error.textContent = "";

      for (const j of s.joints) {
        const m = marks[j];
        const [lo, hi] = s.limits[j];
        const p = s.positions[j];
        const out = p !== undefined && (p < lo - 1 || p > hi + 1);  // jog zablokowany na serwerze
        m.v.textContent = p === undefined ? "-" : p.toFixed(1) + (out ? " POZA ZAKRESEM" : "");
        m.v.style.color = out ? "var(--armp-bad)" : "";
        m.lo.textContent = lo.toFixed(0);
        m.hi.textContent = hi.toFixed(0);
        placeMark(m.p, p, lo, hi);
        placeMark(m.s, s.setpoint[j], lo, hi);
      }
      setEnabled(s.homed);
    }

    async function poll() {
      let delay = 500;
      try {
        const r = await fetch(base + "/api/state", { cache: "no-store" });
        render(await r.json());
        if (sendError === OFFLINE) showError("");
      } catch (e) {
        el.dot.className = "armp-dot";
        el.st.textContent = OFFLINE;
        setEnabled(false);
        delay = 2000;  // serwer ramienia wylaczony: nie zasypuj konsoli bledami
      }
      setTimeout(poll, delay);
    }

    root.querySelectorAll("[data-cmd]").forEach((b) => {
      b.onclick = () => send({ cmd: b.dataset.cmd });
    });
    q(".armp-add").onclick = async () => {
      const ok = await send({
        cmd: "add_point", label: el.label.value.trim(), seconds: Number(el.secs.value),
        check_gripper: el.check.checked,
      }, true);
      if (ok) { el.label.value = ""; el.check.checked = false; }
    };
    q(".armp-zero").onclick = () => {
      if (window.confirm("Ramię stoi wyprostowane poziomo do przodu? Obecny odczyt stanie się zerem URDF.")) send({ cmd: "urdf_zero" }, true);
    };
    q(".armp-drop").onclick = () => send({ cmd: "drop_point" }, true);
    q(".armp-clear").onclick = () => {
      if (window.confirm("Wyczyscic szkic ruchu?")) send({ cmd: "clear_points" }, true);
    };
    q(".armp-save").onclick = async () => {
      const ok = await send({ cmd: "save_motion", name: el.mname.value.trim(), overwrite: el.over.checked }, true);
      if (ok) el.over.checked = false;
    };
    const stop = () => send({ cmd: "stop" });
    q(".armp-stop").onclick = stop;
    if (opts.keyboardStop) {
      document.addEventListener("keydown", (e) => {
        if (e.code === "Space") { e.preventDefault(); stop(); }
      });
      // spacja na przycisku z fokusem inaczej "kliknelaby" go przy puszczeniu klawisza
      document.addEventListener("keyup", (e) => { if (e.code === "Space") e.preventDefault(); });
    }
    poll();
    return { stop, motions: () => lastMotions };
  }

  window.mountArmPanel = mountArmPanel;
})();
