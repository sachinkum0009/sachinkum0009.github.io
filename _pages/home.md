---
layout: page
title: Home
permalink: /
comments: false
---

{% if page.url == "/" %}

<!-- Modern Hero Section -->
<section class="hero" style="padding: 80px 0 40px 0; text-align: center; background: linear-gradient(120deg, #6a11cb 0%, #2575fc 100%); color: #fff; position: relative;">
    <div style="display: flex; flex-direction: column; align-items: center;">
        <img src="https://avatars.githubusercontent.com/u/19277194?v=4" alt="Sachin Kumar" style="width: 140px; height: 140px; border-radius: 50%; box-shadow: 0 4px 24px rgba(0,0,0,0.15); margin-bottom: 24px; border: 4px solid #fff;">
        <h1 style="font-size: 3em; font-weight: 700; margin-bottom: 0.2em; letter-spacing: 1px;">Sachin Kumar</h1>
        <span id="typed-text" style="font-size: 1.5em; font-weight: 400; margin-bottom: 1em; display: block; min-height: 2em;"></span>
        <a href="/blog" style="display: inline-block; padding: 14px 38px; background: #fff; color: #2575fc; border-radius: 30px; font-weight: bold; text-decoration: none; box-shadow: 0 2px 12px rgba(0,0,0,0.10); transition: background 0.2s, color 0.2s; font-size: 1.1em; margin-top: 10px;">Read My Blog</a>
    </div>
    <script>
    document.addEventListener("DOMContentLoaded", function() {
        var phrases = [
            "Developer. Writer. Creator.",
            "Building cool things with code.",
            "Sharing knowledge & ideas.",
            "Always learning, always growing."
        ];
        var el = document.getElementById("typed-text");
        var idx = 0, charIdx = 0, typing = true;
        function type() {
            if (typing) {
                if (charIdx < phrases[idx].length) {
                    el.textContent += phrases[idx][charIdx++];
                    setTimeout(type, 50);
                } else {
                    typing = false;
                    setTimeout(type, 1200);
                }
            } else {
                if (charIdx > 0) {
                    el.textContent = el.textContent.slice(0, -1);
                    charIdx--;
                    setTimeout(type, 25);
                } else {
                    typing = true;
                    idx = (idx + 1) % phrases.length;
                    setTimeout(type, 400);
                }
            }
        }
        el.textContent = "";
        type();
    });
    </script>
</section>

<!-- About Section -->
<section class="about" style="padding: 40px 0; text-align: center;">
    <div style="max-width: 600px; margin: 0 auto;">
        <div style="background: #fff; border-radius: 18px; box-shadow: 0 4px 24px rgba(0,0,0,0.08); padding: 32px 24px;">
            <h2 style="margin-bottom: 0.5em; font-weight: 600; color: #2575fc;">About Me</h2>
            <p style="font-size: 1.2em; color: #444;">
                Hi! I'm Sachin Kumar, a passionate developer, writer, and lifelong learner.<br>
                I love building cool things, sharing knowledge, and exploring new technologies.<br>
                Here you'll find my latest thoughts, tutorials, and projects.<br>
                <span style="color:#6a11cb;">Let's connect and create something amazing!</span>
            </p>
        </div>
    </div>
</section>

<!-- Social Links -->
<section class="social" style="padding: 30px 0; text-align: center;">
    <h3 style="margin-bottom: 1em; font-weight: 600; color: #2575fc;">Connect with me</h3>
    <div style="display: flex; justify-content: center; gap: 32px; font-size: 2.4em;">
        <a href="https://github.com/sachinkum0009" target="_blank" style="color: #333; transition: color 0.2s;"><i class="fab fa-github" style="background: #fff; border-radius: 50%; padding: 12px; box-shadow: 0 2px 8px rgba(0,0,0,0.08);"></i></a>
        <a href="https://www.linkedin.com/in/sachin-kumar-aaa263151/" target="_blank" style="color: #0077b5; transition: color 0.2s;"><i class="fab fa-linkedin" style="background: #fff; border-radius: 50%; padding: 12px; box-shadow: 0 2px 8px rgba(0,0,0,0.08);"></i></a>
        <a href="mailto:sachinkum0009@gmail.com" style="color: #ea4335; transition: color 0.2s;"><i class="fa fa-envelope" style="background: #fff; border-radius: 50%; padding: 12px; box-shadow: 0 2px 8px rgba(0,0,0,0.08);"></i></a>
    </div>
    <style>
        .social a:hover { color: #6a11cb !important; }
    </style>
</section>

<!-- Highlights Section -->
<section class="highlights" style="padding: 40px 0; text-align: center; background: #f7f7f7;">
    <div style="max-width: 800px; margin: 0 auto;">
        <h2 style="margin-bottom: 0.5em; font-weight: 600; color: #2575fc;">Highlights</h2>
        <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 24px;">
            <div style="background: #fff; border-radius: 16px; box-shadow: 0 2px 12px rgba(0,0,0,0.07); padding: 24px 32px; min-width: 220px; font-size: 1.1em; color: #333;">
                🚀 Built several open-source projects on <a href="https://github.com/sachinkum0009" target="_blank" style="color:#2575fc;">GitHub</a>
            </div>
            <div style="background: #fff; border-radius: 16px; box-shadow: 0 2px 12px rgba(0,0,0,0.07); padding: 24px 32px; min-width: 220px; font-size: 1.1em; color: #333;">
                📝 Published tutorials and articles on web development
            </div>
            <div style="background: #fff; border-radius: 16px; box-shadow: 0 2px 12px rgba(0,0,0,0.07); padding: 24px 32px; min-width: 220px; font-size: 1.1em; color: #333;">
                💡 Always learning and experimenting with new tech
            </div>
        </div>
    </div>
</section>

{% endif %}
