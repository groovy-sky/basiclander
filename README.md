# basiclander
[![CI status](https://gitlab.com/tslocum/basiclander/badges/master/pipeline.svg)](https://gitlab.com/tslocum/basiclander/commits/master)
[![Donate](https://img.shields.io/liberapay/receives/rocketnine.space.svg?logo=liberapay)](https://liberapay.com/rocketnine.space)

[Lunar Lander](https://en.wikipedia.org/wiki/Lunar_Lander_(1979_video_game)) clone

**Note:** This game was created for the [LibreJam December 2020 game jam](https://leagueh.xyz/en/jam.html).
It is playable and winnable, however it is minimalist to say the least.

## Screenshot

[![Screenshot](https://gitlab.com/tslocum/basiclander/-/raw/master/screenshot.png)](https://gitlab.com/tslocum/basiclander/-/raw/master/screenshot.png)

## Controls

- Rotate counter-clockwise: **Left**
- Rotate clockwise: **Right**
- Thrust: **Space**

## Play via SSH

```bash
ssh rocketnine.space -p 20067
```

## Compile

basiclander is written in [Go](https://golang.org). Run the following command to
download and build basiclander from source.

```bash
go get gitlab.com/tslocum/basiclander
```

The resulting binary is available as `~/go/bin/basiclander`.

## Support

Please share issues and suggestions [here](https://gitlab.com/tslocum/basiclander/issues).

## Dependencies

- [box2d](https://github.com/ByteArena/box2d) - Physics engine
- [tcell](https://github.com/gdamore/tcell) - Low-level terminal interface
- [cview](https://gitlab.com/tslocum/cview) - High-level terminal interface
