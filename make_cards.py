"""Opening and closing cards for the demo video (1280x720)."""
from PIL import Image, ImageDraw, ImageFont

F = "/System/Library/Fonts/Helvetica.ttc"
f = lambda s, b=False: ImageFont.truetype(F, s, index=1 if b else 0)
BG, FG, GREY, YEL = (22, 23, 28), (240, 240, 236), (160, 164, 174), (255, 198, 26)


def card(path, lines):
    im = Image.new("RGB", (1280, 720), BG)
    d = ImageDraw.Draw(im)
    for xy, text, font, col in lines:
        d.text(xy, text, font=font, fill=col)
    im.save(path)


card("brand/card_quote.png", [
    ((110, 170), "After strokes left her with sight loss, Jennie got lost on a short walk near her home.", f(26), GREY),
    ((110, 240), "\u201cWhen I got lost, I was too scared to go out.", f(46, True), FG),
    ((110, 302), "I lost all my confidence.\u201d", f(46, True), FG),
    ((110, 400), "What got her out again was a rehabilitation specialist walking her routes", f(28), FG),
    ((110, 440), "with her, until she could go out alone.", f(28), FG),
    ((110, 530), "Jennie  \u00b7  Guide Dogs Annual Report 2025", f(20), GREY),
])
card("brand/card_intro.png", [
    ((90, 110), "Most people don\u2019t get Jennie\u2019s help in time.", f(42, True), FG),
    ((90, 190), "Rehab:", f(28, True), YEL), ((300, 192), "85% of English councils miss the 28-day target for a rehabilitation assessment.", f(24), FG),
    ((90, 236), "Guide dogs:", f(28, True), YEL), ((300, 238), "980 people are waiting; 553 were matched last year.", f(24), FG),
    ((90, 282), "Station staff:", f(28, True), YEL), ((300, 284), "even help booked in advance reliably turns up for only 44% of blind passengers.", f(24), FG),
    ((90, 400), "WAYFINDER", f(64, True), YEL),
    ((90, 480), "A robot guide that walks your routes with you.", f(32), FG),
    ((90, 524), "You lead when you can. It leads when you need.", f(32, True), FG),
    ((90, 640), "Sources: RNIB Life on Pause, Oct 2025; Guide Dogs Annual Report 2025; RNIB rail report, Jan 2026", f(16), GREY),
])
card("brand/card_end.png", [
    ((90, 100), "80 random journeys: crowds, a drop, a sign put down mid-run", f(26), GREY),
    ((90, 150), "80 arrived.  0 times it moved into anyone.", f(40, True), FG),
    ((90, 206), "Never closer than 0.83 m to the drop.", f(40, True), FG),
    ((90, 290), "On a remembered route it handed the lead to the user in 29 of 40 runs,", f(28), FG),
    ((90, 330), "and took it back, saying why, when they needed it.", f(28), FG),
    ((90, 410), "One CPU core. Decisions in under 2 ms. No GPU, no network, speech on-device.", f(26), GREY),
    ((90, 500), "WAYFINDER", f(56, True), YEL),
    ((90, 572), "You lead when you can. It leads when you need.", f(28), FG),
    ((90, 640), "github.com/anushaisawesome/wayfinder", f(20), GREY),
])
