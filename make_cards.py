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


card("brand/card_intro.png", [
    ((90, 120), "Every day, 250 people in the UK start to lose their sight.", f(34, True), FG),
    ((90, 185), "980 are waiting for a guide dog.", f(30), FG),
    ((90, 232), "Only 44% of blind rail passengers are reliably met by station assistance.", f(30), FG),
    ((90, 280), "“I know the assistance is going to fail. No one meets me.”  Paul Goddard", f(24), GREY),
    ((90, 400), "WAYFINDER", f(64, True), YEL),
    ((90, 480), "A robot guide. You lead when you can; it leads when you need,", f(32), FG),
    ((90, 524), "reading your grip and the route, and saying why.", f(32), FG),
    ((90, 640), "Sources: RNIB key statistics; Guide Dogs Annual Report 2025; RNIB rail report, Jan 2026", f(16), GREY),
])
card("brand/card_end.png", [
    ((90, 110), "80 random journeys, new and remembered routes", f(34, True), FG),
    ((90, 180), "79 arrived  ·  0 times the robot moved into anyone", f(30), FG),
    ((90, 226), "never closer than 0.78 m to the drop", f(30), FG),
    ((90, 272), "on a known route it handed the lead to the user in 27 of 40 runs, only once the drop was behind", f(24), GREY),
    ((90, 370), "Runs on one CPU core: decision under 2 ms, no GPU, no network, speech on-device", f(28), FG),
    ((90, 470), "WAYFINDER", f(48, True), YEL),
    ((90, 535), "github.com/anushaisawesome/wayfinder", f(26), GREY),
])
