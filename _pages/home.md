---
layout: page
title: Home
permalink: /
comments: false
---

{% if page.url == "/" %}

<!-- Hero Section
================================================== -->
<section class="hero" style="padding: 60px 0; text-align: center; background: linear-gradient(135deg, #6a11cb 0%, #2575fc 100%); color: #fff;">
    <h1 style="font-size: 3em; margin-bottom: 0.3em;">Sachin Kumar</h1>
    <p style="font-size: 1.5em; margin-bottom: 1em;">Welcome to my personal website & blog</p>
    <a href="/blog" style="display: inline-block; padding: 12px 32px; background: #fff; color: #2575fc; border-radius: 30px; font-weight: bold; text-decoration: none; box-shadow: 0 2px 8px rgba(0,0,0,0.08); transition: background 0.2s;">Read My Blog</a>
</section>

<!-- About Section
================================================== -->
<section class="about" style="padding: 40px 0; text-align: center;">
    <div style="max-width: 700px; margin: 0 auto;">
        <h2 style="margin-bottom: 0.5em;">About Me</h2>
        <p style="font-size: 1.2em; color: #444;">
            Hi! I'm Sachin Kumar, a passionate developer, writer, and lifelong learner. I love building cool things, sharing knowledge, and exploring new technologies. Here you'll find my latest thoughts, tutorials, and projects.
        </p>
    </div>
</section>

<!-- Social Links
================================================== -->
<section class="social" style="padding: 30px 0; text-align: center;">
    <h3 style="margin-bottom: 1em;">Connect with me</h3>
    <div style="font-size: 2em;">
        <a href="https://github.com/sachinkum0009" target="_blank" style="margin: 0 15px; color: #333;"><i class="fa fa-github"></i></a>
        <a href="https://twitter.com/sachinkum0009" target="_blank" style="margin: 0 15px; color: #1da1f2;"><i class="fa fa-twitter"></i></a>
        <a href="mailto:sachinkumar.ar97@gmail.com" style="margin: 0 15px; color: #ea4335;"><i class="fa fa-envelope"></i></a>
        <!-- Add more social links as needed -->
    </div>
</section>

<!-- Highlights Section (Optional)
================================================== -->
<section class="highlights" style="padding: 40px 0; text-align: center; background: #f7f7f7;">
    <div style="max-width: 700px; margin: 0 auto;">
        <h2 style="margin-bottom: 0.5em;">Highlights</h2>
        <ul style="list-style: none; padding: 0; font-size: 1.1em; color: #333;">
            <li style="margin-bottom: 0.7em;">🚀 Built several open-source projects on GitHub</li>
            <li style="margin-bottom: 0.7em;">📝 Published tutorials and articles on web development</li>
            <li style="margin-bottom: 0.7em;">💡 Always learning and experimenting with new tech</li>
        </ul>
    </div>
</section>

{% endif %}
