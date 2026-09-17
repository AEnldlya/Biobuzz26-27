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
 * (build/sim/&lt;name&gt;.html): the field from above, the robot, the turret and where it should
 * be pointing, pollen, shots, and the Driver Station telemetry, with a play / scrub control.
 */
public class SimReport {
    private static final class Frame {
        double t, x, y, heading, turretDeg, aimDeg, speed;
        int balls, shots, scored;
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
        f.label = world.label();
        f.telemetry = world.telemetry.toString();
        f.collected = new boolean[world.pollen.size()];
        for (int i = 0; i < f.collected.length; i++) {
            f.collected[i] = world.pollen.get(i).collected;
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
            out.println("t,x,y,headingDeg,turretDeg,trueAimDeg,errorDeg,speedInS,balls,shots,scored,label");
            for (Frame f : frames) {
                double err = f.turretDeg - f.aimDeg;
                err = ((err + 180) % 360 + 360) % 360 - 180;
                out.printf(Locale.US, "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.1f,%d,%d,%d,%s%n",
                        f.t, f.x, f.y, Math.toDegrees(f.heading), f.turretDeg, f.aimDeg, err, f.speed,
                        f.balls, f.shots, f.scored, f.label);
            }
        }

        StringBuilder data = new StringBuilder();
        data.append("{\"alliance\":\"").append(world.alliance).append("\",");
        Pose cell = Field.cellAimPoint(world.alliance, world.upCell);
        data.append("\"cell\":[").append(cell.x()).append(',').append(cell.y()).append("],");
        data.append("\"pollen\":[");
        for (int i = 0; i < world.pollen.size(); i++) {
            SimWorld.PollenPiece pp = world.pollen.get(i);
            if (i > 0) data.append(',');
            data.append(String.format(Locale.US, "[%.1f,%.1f,\"%s\"]", pp.x, pp.y, pp.color));
        }
        data.append("],\"shots\":[");
        for (int i = 0; i < world.shots.size(); i++) {
            SimWorld.Shot s = world.shots.get(i);
            if (i > 0) data.append(',');
            data.append(String.format(Locale.US, "[%.2f,%.1f,%.1f,%s,%.1f,%.0f]", s.timeS, s.landX, s.landY, s.scored, s.missIn, s.rpm));
        }
        data.append("],\"frames\":[");
        for (int i = 0; i < frames.size(); i++) {
            Frame f = frames.get(i);
            if (i > 0) data.append(',');
            data.append(String.format(Locale.US, "{\"t\":%.2f,\"x\":%.2f,\"y\":%.2f,\"h\":%.4f,\"tu\":%.1f,\"aim\":%.1f,\"v\":%.1f,\"b\":%d,\"s\":%d,\"sc\":%d,\"l\":\"%s\",\"tel\":\"%s\",\"c\":[",
                    f.t, f.x, f.y, f.heading, f.turretDeg, f.aimDeg, f.speed, f.balls, f.shots, f.scored, esc(f.label), esc(f.telemetry)));
            for (int j = 0; j < f.collected.length; j++) {
                if (j > 0) data.append(',');
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
            + ".big{font-size:18px;margin:6px 0}</style></head><body><div id=\"left\">"
            + "<canvas id=\"c\" width=\"576\" height=\"576\"></canvas><div><button id=\"play\">Play</button>"
            + "<input id=\"slider\" type=\"range\" min=\"0\" value=\"0\"><span id=\"time\"></span></div>"
            + "<div>speed <select id=\"speed\"><option>0.25</option><option>0.5</option><option selected>1</option><option>2</option><option>4</option></select></div>"
            + "</div><div id=\"right\"><div class=\"big\" id=\"label\"></div><div id=\"stats\"></div><h3>Driver Station</h3><pre id=\"tel\"></pre>"
            + "<h3>Shots</h3><pre id=\"shots\"></pre></div><script>\n";

    private static final String HTML_TAIL = "const S = 4; // px per inch\n"
            + "const c = document.getElementById('c'), ctx = c.getContext('2d');\n"
            + "const slider = document.getElementById('slider'); slider.max = DATA.frames.length - 1;\n"
            + "function fx(x){return x*S;} function fy(y){return c.height - y*S;}\n"
            + "function draw(i){ const f = DATA.frames[i]; ctx.clearRect(0,0,c.width,c.height);\n"
            + "  ctx.strokeStyle='#333'; for(let k=0;k<=6;k++){ctx.beginPath();ctx.moveTo(fx(k*24),0);ctx.lineTo(fx(k*24),c.height);ctx.stroke();ctx.beginPath();ctx.moveTo(0,fy(k*24));ctx.lineTo(c.width,fy(k*24));ctx.stroke();}\n"
            + "  ctx.fillStyle='#844'; ctx.fillRect(fx(72-12.75-6),fy(72+18),12*S,36*S); ctx.fillStyle='#448'; ctx.fillRect(fx(72+12.75-6),fy(72+18),12*S,36*S);\n"
            + "  ctx.fillStyle='#ff0'; ctx.beginPath(); ctx.arc(fx(DATA.cell[0]),fy(DATA.cell[1]),8*S,0,7); ctx.stroke(); ctx.strokeStyle='#ff0'; ctx.stroke();\n"
            + "  DATA.pollen.forEach((p,j)=>{ if(f.c[j]) return; ctx.fillStyle = p[2]=='PURPLE'?'#b6f':(p[2]=='GREEN'?'#6f6':'#fd5'); ctx.beginPath(); ctx.arc(fx(p[0]),fy(p[1]),2.5*S,0,7); ctx.fill(); });\n"
            + "  ctx.strokeStyle='#5af'; ctx.lineWidth=1; ctx.beginPath(); for(let k=0;k<=i;k++){const g=DATA.frames[k]; if(k==0) ctx.moveTo(fx(g.x),fy(g.y)); else ctx.lineTo(fx(g.x),fy(g.y));} ctx.stroke();\n"
            + "  DATA.shots.forEach(s=>{ if(s[0]>f.t) return; ctx.fillStyle = s[3]?'#3f3':'#f33'; ctx.beginPath(); ctx.arc(fx(s[1]),fy(s[2]),1.5*S,0,7); ctx.fill(); });\n"
            + "  ctx.save(); ctx.translate(fx(f.x),fy(f.y)); ctx.rotate(-f.h); ctx.fillStyle='#ccc'; ctx.fillRect(-9*S,-9*S,18*S,18*S); ctx.fillStyle='#f80'; ctx.fillRect(6*S,-3*S,3*S,6*S);\n"
            + "  ctx.rotate(f.aim*Math.PI/180); ctx.strokeStyle='#ff0'; ctx.lineWidth=1; ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(60*S,0); ctx.stroke();\n"
            + "  ctx.rotate((f.tu-f.aim)*Math.PI/180); ctx.strokeStyle='#f00'; ctx.lineWidth=3; ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(24*S,0); ctx.stroke(); ctx.restore();\n"
            + "  let err=f.tu-f.aim; err=((err+180)%360+360)%360-180;\n"
            + "  document.getElementById('time').textContent = ' ' + f.t.toFixed(2) + ' s';\n"
            + "  document.getElementById('label').textContent = f.l;\n"
            + "  document.getElementById('stats').innerHTML = 'pose ' + f.x.toFixed(1) + ', ' + f.y.toFixed(1) + ', ' + (f.h*180/Math.PI).toFixed(1) + '&deg;  speed ' + f.v.toFixed(1) + ' in/s<br>turret ' + f.tu.toFixed(1) + '&deg; / should be ' + f.aim.toFixed(1) + '&deg; (error ' + err.toFixed(2) + '&deg;)<br>balls in robot ' + f.b + '  shots ' + f.s + '  scored ' + f.sc;\n"
            + "  document.getElementById('tel').textContent = f.tel;\n"
            + "  document.getElementById('shots').textContent = DATA.shots.filter(s=>s[0]<=f.t).map(s=>s[0].toFixed(2)+' s  '+(s[3]?'SCORED':'miss  ')+' by '+s[4].toFixed(1)+' in at '+s[5]+' rpm').join('\\n');\n"
            + "}\n"
            + "let playing=false, idx=0, last=0;\n"
            + "slider.oninput = ()=>{ idx=+slider.value; draw(idx); };\n"
            + "document.getElementById('play').onclick = ()=>{ playing=!playing; document.getElementById('play').textContent = playing?'Pause':'Play'; last=performance.now(); if(playing) requestAnimationFrame(tick); };\n"
            + "function tick(now){ if(!playing) return; const sp=+document.getElementById('speed').value; const dt=(now-last)/1000*sp; last=now;\n"
            + "  const target = DATA.frames[idx].t + dt; while(idx < DATA.frames.length-1 && DATA.frames[idx+1].t <= target) idx++; if(idx>=DATA.frames.length-1){playing=false;document.getElementById('play').textContent='Play';}\n"
            + "  slider.value=idx; draw(idx); requestAnimationFrame(tick); }\n"
            + "draw(0);\n</script></body></html>\n";
}
