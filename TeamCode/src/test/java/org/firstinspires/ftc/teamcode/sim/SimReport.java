package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Field;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Records the simulation and writes it out as a CSV and a self-contained HTML replay
 * (build/sim/&lt;name&gt;.html): the BIOBUZZ field from above (HIVE frame, both CELLS with the
 * upward one highlighted, FLOWERS, LOADING ZONE, GARDEN), the robot, the turret and the
 * bearing to the CELL, POLLEN and NECTAR, shots, AUTO points, and the Driver Station
 * telemetry, with a play / scrub control.
 */
public class SimReport {
    private static final class Frame {
        double t, x, y, heading, turretDeg, aimDeg, speed;
        int balls, shots, scored, tips, points, up;
        String label, telemetry;
        boolean[] collected;
    }

    private final SimWorld world;
    private final List<Frame> frames = new ArrayList<>();

    public SimReport(SimWorld world) {
        this.world = world;
    }

    void record() {
        Frame f = new Frame();
        Pose p = world.robot.truePose();
        f.t = world.robot.simTimeS();
        f.x = p.x();
        f.y = p.y();
        f.heading = p.heading();
        f.turretDeg = world.turret.ringDeg();
        f.aimDeg = world.trueAimDeg();
        f.speed = world.robot.speedInS();
        f.balls = world.ballsInRobot;
        f.shots = world.shots.size();
        f.scored = world.ballsScored;
        f.tips = world.tips;
        f.points = world.autoPoints();
        f.up = world.upCell == Field.CellSide.FAR ? 1 : 0;
        f.label = world.label();
        f.telemetry = world.telemetry.toString();
        f.collected = new boolean[world.pieces.size()];
        for (int i = 0; i < f.collected.length; i++) {
            f.collected[i] = world.pieces.get(i).collected;
        }
        frames.add(f);
    }

    public int frameCount() {
        return frames.size();
    }

    private static String esc(String s) {
        return s.replace("\\", "\\\\").replace("\"", "\\\"").replace("\n", "\\n").replace("</", "<\\/");
    }

    public File write(String name) throws IOException {
        File dir = new File("build/sim");
        dir.mkdirs();
        File csv = new File(dir, name + ".csv");
        try (PrintWriter out = new PrintWriter(new FileWriter(csv))) {
            out.println("t,x,y,headingDeg,turretDeg,trueAimDeg,errorDeg,speedInS,balls,shots,scored,tips,autoPoints,upCell,label");
            for (Frame f : frames) {
                double err = f.turretDeg - f.aimDeg;
                err = ((err + 180) % 360 + 360) % 360 - 180;
                out.printf(Locale.US, "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.1f,%d,%d,%d,%d,%d,%s,%s%n",
                        f.t, f.x, f.y, Math.toDegrees(f.heading), f.turretDeg, f.aimDeg, err, f.speed,
                        f.balls, f.shots, f.scored, f.tips, f.points, f.up == 1 ? "FAR" : "AUDIENCE", f.label);
            }
        }

        StringBuilder data = new StringBuilder();
        data.append("{\"alliance\":\"").append(world.alliance).append("\",");
        data.append(String.format(Locale.US, "\"hiveX\":%.2f,\"aimOffset\":%.2f,\"frameX\":%.2f,\"frameY\":%.2f,",
                Field.hiveX(world.alliance), Field.CELL_AIM_OFFSET, Field.HIVE_FRAME_X_IN, Field.HIVE_FRAME_Y_IN));
        data.append(String.format(Locale.US, "\"mouthW\":%.2f,\"mouthD\":%.2f,", Field.CELL_MOUTH_WIDTH_IN, Field.CELL_DEPTH_IN));
        Field.Rect lz = Field.loadingZone(world.alliance);
        Field.Rect gd = Field.garden(world.alliance);
        data.append(String.format(Locale.US, "\"loading\":[%.1f,%.1f,%.1f,%.1f],\"garden\":[%.1f,%.1f,%.1f,%.1f],",
                lz.x0, lz.y0, lz.x1, lz.y1, gd.x0, gd.y0, gd.x1, gd.y1));
        data.append("\"flowers\":[[2.3,48],[48,141.7],[141.7,96],[96,2.3]],");
        data.append("\"pieces\":[");
        for (int i = 0; i < world.pieces.size(); i++) {
            SimWorld.Piece pp = world.pieces.get(i);
            if (i > 0) {
                data.append(',');
            }
            data.append(String.format(Locale.US, "[%.1f,%.1f,\"%s\"]", pp.x, pp.y, pp.type));
        }
        data.append("],\"shots\":[");
        for (int i = 0; i < world.shots.size(); i++) {
            SimWorld.Shot s = world.shots.get(i);
            if (i > 0) {
                data.append(',');
            }
            data.append(String.format(Locale.US, "[%.2f,%.1f,%.1f,%s,%.1f,%.0f,%.0f]",
                    s.timeS, s.landX, s.landY, s.scored, s.missIn, s.rpm, s.distanceIn));
        }
        data.append("],\"frames\":[");
        for (int i = 0; i < frames.size(); i++) {
            Frame f = frames.get(i);
            if (i > 0) {
                data.append(',');
            }
            data.append(String.format(Locale.US,
                    "{\"t\":%.2f,\"x\":%.2f,\"y\":%.2f,\"h\":%.4f,\"tu\":%.1f,\"aim\":%.1f,\"v\":%.1f,\"b\":%d,\"s\":%d,\"sc\":%d,\"tips\":%d,\"pts\":%d,\"up\":%d,\"l\":\"%s\",\"tel\":\"%s\",\"c\":[",
                    f.t, f.x, f.y, f.heading, f.turretDeg, f.aimDeg, f.speed, f.balls, f.shots, f.scored,
                    f.tips, f.points, f.up, esc(f.label), esc(f.telemetry)));
            for (int j = 0; j < f.collected.length; j++) {
                if (j > 0) {
                    data.append(',');
                }
                data.append(f.collected[j] ? 1 : 0);
            }
            data.append("]}");
        }
        data.append("]}");

        File html = new File(dir, name + ".html");
        try (PrintWriter out = new PrintWriter(new FileWriter(html))) {
            out.print(HTML_HEAD.replace("__TITLE__", name));
            out.print("const DATA = ");
            out.print(data);
            out.print(";\n");
            out.print(HTML_TAIL);
        }
        return html;
    }

    private static final String HTML_HEAD = "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><title>__TITLE__</title>"
            + "<style>body{font-family:system-ui,sans-serif;margin:0;background:#111;color:#eee;display:flex;height:100vh}"
            + "#left{padding:12px}#right{padding:12px;flex:1;overflow:auto}canvas{background:#1d1d1d;border:1px solid #444}"
            + "pre{font-size:12px;white-space:pre-wrap}button{margin-right:6px}input[type=range]{width:520px}"
            + ".big{font-size:18px;margin:6px 0}.k{color:#8cf}</style></head><body><div id=\"left\">"
            + "<canvas id=\"c\" width=\"576\" height=\"576\"></canvas><div><button id=\"play\">Play</button>"
            + "<input id=\"slider\" type=\"range\" min=\"0\" value=\"0\"><span id=\"time\"></span></div>"
            + "<div>speed <select id=\"speed\"><option>0.25</option><option>0.5</option><option selected>1</option><option>2</option><option>4</option></select>"
            + " &nbsp; <span class=\"k\">yellow POLLEN, red/blue NECTAR, blue dots FLOWERS, boxed CELL is up</span></div>"
            + "</div><div id=\"right\"><div class=\"big\" id=\"label\"></div><div id=\"stats\"></div><h3>Driver Station</h3><pre id=\"tel\"></pre>"
            + "<h3>Shots</h3><pre id=\"shots\"></pre></div><script>\n";

    private static final String HTML_TAIL = "const S = 4; // px per inch\n"
            + "const c = document.getElementById('c'), ctx = c.getContext('2d');\n"
            + "const slider = document.getElementById('slider'); slider.max = DATA.frames.length - 1;\n"
            + "function fx(x){return x*S;} function fy(y){return c.height - y*S;}\n"
            + "function draw(i){ const f = DATA.frames[i]; ctx.clearRect(0,0,c.width,c.height);\n"
            + "  ctx.strokeStyle='#333'; ctx.lineWidth=1; for(let k=0;k<=6;k++){ctx.beginPath();ctx.moveTo(fx(k*24),0);ctx.lineTo(fx(k*24),c.height);ctx.stroke();ctx.beginPath();ctx.moveTo(0,fy(k*24));ctx.lineTo(c.width,fy(k*24));ctx.stroke();}\n"
            + "  const L=DATA.loading, G=DATA.garden;\n"
            + "  ctx.fillStyle='rgba(120,70,70,0.45)'; ctx.fillRect(fx(L[0]),fy(L[3]),(L[2]-L[0])*S,(L[3]-L[1])*S);\n"
            + "  ctx.fillStyle='rgba(180,90,90,0.9)'; ctx.fillRect(fx(G[0]),fy(G[3]),(G[2]-G[0])*S,(G[3]-G[1])*S);\n"
            + "  ctx.strokeStyle='#666'; ctx.strokeRect(fx(72-DATA.frameX/2),fy(72+DATA.frameY/2),DATA.frameX*S,DATA.frameY*S);\n"
            + "  const hx=DATA.hiveX, ao=DATA.aimOffset, mw=DATA.mouthW, md=DATA.mouthD;\n"
            + "  [[72-ao,f.up==0],[72+ao,f.up==1]].forEach(function(e){ const cy=e[0], isUp=e[1];\n"
            + "    ctx.fillStyle = isUp ? '#c44' : '#533'; ctx.fillRect(fx(hx-mw/2),fy(cy+md/2),mw*S,md*S);\n"
            + "    if(isUp){ ctx.strokeStyle='#ff0'; ctx.lineWidth=2; ctx.strokeRect(fx(hx-mw/2),fy(cy+md/2),mw*S,md*S); ctx.lineWidth=1; } });\n"
            + "  ctx.fillStyle='#69c'; DATA.flowers.forEach(function(fl){ ctx.beginPath(); ctx.arc(fx(fl[0]),fy(fl[1]),2.5*S,0,7); ctx.fill(); });\n"
            + "  DATA.pieces.forEach(function(p,j){ if(j>=f.c.length||f.c[j]) return; ctx.fillStyle = p[2]=='POLLEN'?'#fd3':(p[2]=='RED_NECTAR'?'#f55':'#59f'); ctx.beginPath(); ctx.arc(fx(p[0]),fy(p[1]),(p[2]=='POLLEN'?1.4:1.81)*S,0,7); ctx.fill(); });\n"
            + "  ctx.strokeStyle='#5af'; ctx.beginPath(); for(let k=0;k<=i;k++){const g=DATA.frames[k]; if(k==0) ctx.moveTo(fx(g.x),fy(g.y)); else ctx.lineTo(fx(g.x),fy(g.y));} ctx.stroke();\n"
            + "  DATA.shots.forEach(function(s){ if(s[0]>f.t) return; ctx.fillStyle = s[3]?'#3f3':'#f33'; ctx.beginPath(); ctx.arc(fx(s[1]),fy(s[2]),1.5*S,0,7); ctx.fill(); });\n"
            + "  ctx.save(); ctx.translate(fx(f.x),fy(f.y)); ctx.rotate(-f.h); ctx.fillStyle='#ccc'; ctx.fillRect(-6*S,-6*S,12*S,12*S); ctx.fillStyle='#f80'; ctx.fillRect(4*S,-3*S,2*S,6*S);\n"
            + "  ctx.rotate(f.aim*Math.PI/180); ctx.strokeStyle='#ff0'; ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(40*S,0); ctx.stroke();\n"
            + "  ctx.rotate((f.tu-f.aim)*Math.PI/180); ctx.strokeStyle='#f00'; ctx.lineWidth=3; ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(20*S,0); ctx.stroke(); ctx.restore();\n"
            + "  let err=f.tu-f.aim; err=((err+180)%360+360)%360-180;\n"
            + "  document.getElementById('time').textContent = ' ' + f.t.toFixed(2) + ' s';\n"
            + "  document.getElementById('label').textContent = f.l;\n"
            + "  document.getElementById('stats').innerHTML = 'pose ' + f.x.toFixed(1) + ', ' + f.y.toFixed(1) + ', ' + (f.h*180/Math.PI).toFixed(1) + '&deg;  speed ' + f.v.toFixed(1) + ' in/s'\n"
            + "    + '<br>turret ' + f.tu.toFixed(1) + '&deg; / geometric bearing to the up CELL ' + f.aim.toFixed(1) + '&deg; (diff ' + err.toFixed(2) + '&deg;; while driving the turret leads the CELL on purpose, see Turret.LEAD_GAIN)'\n"
            + "    + '<br>balls in robot ' + f.b + '  shots ' + f.s + '  in the CELL ' + f.sc + '  HIVE tips ' + f.tips + '  up CELL ' + (f.up?'FAR':'AUDIENCE')\n"
            + "    + '<br><b>AUTO points ' + f.pts + '</b>';\n"
            + "  document.getElementById('tel').textContent = f.tel;\n"
            + "  document.getElementById('shots').textContent = DATA.shots.filter(function(s){return s[0]<=f.t;}).map(function(s){return s[0].toFixed(2)+' s  '+(s[3]?'IN    ':'miss  ')+s[4].toFixed(1)+' in from the mouth centre, '+s[5]+' rpm from '+s[6]+' in';}).join('\\n');\n"
            + "}\n"
            + "let playing=false, idx=0, last=0;\n"
            + "slider.oninput = function(){ idx=+slider.value; draw(idx); };\n"
            + "document.getElementById('play').onclick = function(){ playing=!playing; document.getElementById('play').textContent = playing?'Pause':'Play'; last=performance.now(); if(playing) requestAnimationFrame(tick); };\n"
            + "function tick(now){ if(!playing) return; const sp=+document.getElementById('speed').value; const dt=(now-last)/1000*sp; last=now;\n"
            + "  const target = DATA.frames[idx].t + dt; while(idx < DATA.frames.length-1 && DATA.frames[idx+1].t <= target) idx++; if(idx>=DATA.frames.length-1){playing=false;document.getElementById('play').textContent='Play';}\n"
            + "  slider.value=idx; draw(idx); requestAnimationFrame(tick); }\n"
            + "draw(0);\n</script></body></html>\n";
}
