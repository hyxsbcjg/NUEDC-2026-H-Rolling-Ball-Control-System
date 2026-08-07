# Third-party notices

## LibVLC Android 3.7.5

The Android application includes:

- `org.videolan.android:libvlc-all:3.7.5`
- libvlcjni commit `cfe024f6cf9d188f6057d1c8c30d317cb4ba4b1f`
- VLC core commit `ac6c2a405d652b5576128ceb9fec2c342f0e83ec`

VideoLAN publishes libVLC and libvlcjni under the GNU Lesser General Public
License. The packaged native runtime can also contain codec and platform
dependencies with their own licenses.

Source and license references:

- https://code.videolan.org/videolan/libvlcjni
- https://code.videolan.org/videolan/vlc
- https://www.videolan.org/legal.html

This repository currently delivers a debug-signed APK for development and
hardware acceptance. Before public or commercial distribution, generate a
complete dependency license inventory from the pinned VideoLAN build, include
all required license texts and notices, and confirm relinking/source-offer
obligations with qualified counsel.

## Latin Modern Roman 10 Italic

The bottom `fuuu1688` credit uses the Latin Modern Roman 10 Italic OpenType
font from the CTAN Latin Modern package, derived from the Computer Modern
design used by TeX.

- Font file: [`app/src/main/res/font/lmroman10_italic.otf`](app/src/main/res/font/lmroman10_italic.otf)
- Upstream package: https://ctan.org/pkg/lm
- Font SHA-256:
  `C1FCE25075567BB8DBF2151658C3B442690041DB17A2D49FC9E55905EA5B7169`
- License: GUST Font License 1.0

The complete GUST Font License text is packaged at
[`app/src/main/assets/licenses/GUST_FONT_LICENSE.txt`](app/src/main/assets/licenses/GUST_FONT_LICENSE.txt).

## Android and Gradle

The project builds with the Android SDK, Android Gradle Plugin, Gradle, Kotlin,
and JUnit. Their licenses remain with their respective authors. Gradle resolves
these tools from the repositories declared in [`settings.gradle.kts`](settings.gradle.kts).
