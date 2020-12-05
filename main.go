package main

import (
	"bytes"
	"fmt"
	"log"
	"math"
	"strings"
	"time"

	"github.com/gdamore/tcell/v2"
	"gitlab.com/tslocum/cbind"

	"github.com/ByteArena/box2d"
	"gitlab.com/tslocum/cview"
)

var app *cview.Application

var tv *cview.TextView

var actors = make(map[string]*box2d.B2Body)

var updates = make(chan func(), 10)

var lastAT = time.Time{}
var lastLT = time.Time{}

const moonGravity = -1.625 * .1

const thrustMagnitude = 1000.0

const rotateSpeed = 0.005

const debug = true

const (
	startAngle = math.Pi / 3
	startLVX   = 1
	startLVY   = 0
	startX     = 15
	startY     = 32
)

var (
	playerAngle      float64
	playerX, playerY float64
	playerAV         float64
	playerLV         float64
	playerLanded     bool
)

const (
	screenHeight = 32
	screenWidth  = 120
)

var stars = map[int]map[int]string{
	4:   {7: "*"},
	9:   {23: "."},
	11:  {42: "."},
	32:  {16: "."},
	87:  {22: "."},
	58:  {28: "*"},
	72:  {17: "*"},
	102: {25: "*"},
	96:  {9: "."},
	56:  {11: "."},
	25:  {17: "*"},
	42:  {24: "."},
	28:  {14: "."},
}

func setupInput() {
	inputConfig := cbind.NewConfiguration()

	_ = inputConfig.Set("Left", func(ev *tcell.EventKey) *tcell.EventKey {
		updates <- func() {
			if playerLanded {
				return
			}

			v := actors["player"].GetAngularVelocity()
			v -= rotateSpeed
			actors["player"].SetAngularVelocity(v)
			lastAT = time.Now()
		}

		return nil
	})

	_ = inputConfig.Set("Right", func(ev *tcell.EventKey) *tcell.EventKey {
		updates <- func() {
			if playerLanded {
				return
			}

			v := actors["player"].GetAngularVelocity()
			v += rotateSpeed
			actors["player"].SetAngularVelocity(v)
			lastAT = time.Now()
		}

		return nil
	})

	moveFunc :=
		func() {
			updates <- func() {
				angle := actors["player"].GetAngle() - math.Pi/2
				//actors["player"].ApplyLinearImpulseToCenter(box2d.B2Vec2{math.Cos(angle), math.Sin(angle)}, true)
				actors["player"].ApplyLinearImpulse(box2d.B2Vec2{math.Sin(angle) * thrustMagnitude, math.Cos(angle) * thrustMagnitude}, actors["player"].GetWorldCenter(), true)
				lastLT = time.Now()
			}
		}

	_ = inputConfig.Set("Up", func(ev *tcell.EventKey) *tcell.EventKey {
		moveFunc()
		return nil
	})

	_ = inputConfig.Set("Space", func(ev *tcell.EventKey) *tcell.EventKey {
		moveFunc()
		return nil
	})

	app.SetInputCapture(inputConfig.Capture)
}

func renderPoint(area int, x int, y int) string {
	sx, ok := stars[x]
	if ok {
		sy, ok := sx[y]
		if ok {
			return sy
		}
	}
	return " "
}

func renderWorld() {
	px, py := int(math.Floor(playerX)), int(math.Floor(playerY))
	a := playerAngle
	var buf bytes.Buffer
	for y := screenHeight - 1; y >= 0; y-- {
		if y != screenHeight-1 {
			buf.Write([]byte("\n"))
		}
		for x := 0; x < screenWidth; x++ {
			if x == px && y == py {
				if a > 337.5 || a < 22.5 {
					buf.Write([]byte("⬆"))
				} else if a < 67.5 {
					buf.Write([]byte("⬈"))
				} else if a < 112.5 {
					buf.Write([]byte("➡"))
				} else if a < 157.5 {
					buf.Write([]byte("⬊"))
				} else if a < 202.5 {
					buf.Write([]byte("⬇"))
				} else if a < 247.5 {
					buf.Write([]byte("⬋"))
				} else if a < 292.5 {
					buf.Write([]byte("⬅"))
				} else {
					buf.Write([]byte("⬉"))
				}
				continue
			}

			buf.Write([]byte(renderPoint(1, x, y)))
		}
	}
	buf.Write([]byte("\n"))
	buf.Write([]byte("[gray]███▓████████████████▓▓▓██████▓▓████████▓▓▓████████▓▓▓▓█████████████▓▓████████▓████▓▓███████████▓▓██████▓▓▓▓▓██████████[-]"))
	buf.Write([]byte("\n"))
	buf.Write([]byte("\n"))
	if time.Since(lastAT) < 150*time.Millisecond {
		buf.Write([]byte("  [ ROTATIONAL THRUSTERS ] "))
	} else {
		buf.Write([]byte(strings.Repeat(" ", 27)))
	}
	if time.Since(lastLT) < 150*time.Millisecond {
		buf.Write([]byte("  [ POSITIONAL THRUSTERS ] "))
	} else {
		buf.Write([]byte(strings.Repeat(" ", 27)))
	}
	buf.Write([]byte("\n"))

	// Debug
	buf.Write([]byte("\n"))
	buf.Write([]byte(fmt.Sprintf("%.00f,%.00f @ %.02f - Rotation angle: %.00f - Rotation speed: %.02f", playerX, playerY, playerLV, playerAngle, playerAV)))

	tv.SetBytes(buf.Bytes())
}

func runSimulation() {
	// Define the gravity vector.
	gravity := box2d.MakeB2Vec2(0.0, moonGravity)

	// Construct a world object, which will hold and simulate the rigid bodies.
	world := box2d.MakeB2World(gravity)

	{
		bd := box2d.MakeB2BodyDef()
		ground := world.CreateBody(&bd)

		shape := box2d.MakeB2EdgeShape()
		shape.Set(box2d.MakeB2Vec2(-20000.0, 0.0), box2d.MakeB2Vec2(20000.0, -1.0))
		ground.CreateFixture(&shape, 0.0)
		actors["ground"] = ground
	}

	// Circle character
	{
		bd := box2d.MakeB2BodyDef()
		bd.Position.Set(startX, startY)
		bd.LinearVelocity.Set(startLVX, startLVY)
		bd.Type = box2d.B2BodyType.B2_dynamicBody
		bd.FixedRotation = true
		bd.AllowSleep = false

		body := world.CreateBody(&bd)

		body.SetTransform(body.GetPosition(), startAngle)

		shape := box2d.MakeB2CircleShape()
		shape.M_radius = 0.5

		fd := box2d.MakeB2FixtureDef()
		fd.Shape = &shape
		fd.Density = 15103
		body.CreateFixtureFromDef(&fd)
		actors["player"] = body
	}

	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 10 iterations. This provides a high quality simulation
	// in most game scenarios.
	timeStep := 1.0 / 60.0
	velocityIterations := 8
	positionIterations := 3

	t := time.NewTicker(time.Second / 60)
	for {
	UPDATELOOP:
		for {
			select {
			case f := <-updates:
				f()
			default:
				break UPDATELOOP
			}
		}

	WAITLOOP:
		for {
			select {
			case f := <-updates:
				f()
			case <-t.C:
				break WAITLOOP
			}
		}

		// Instruct the world to perform a single step of simulation.
		// It is generally best to keep the time step and iterations fixed.
		//runtime.Breakpoint()
		world.Step(timeStep, velocityIterations, positionIterations)

		list := actors["player"].GetContactList()
		playerLanded = list != nil
		if playerLanded && !debug {
			v := actors["player"].GetLinearVelocity()
			if v.X > 2.5 || v.X < -2.5 || v.Y > 2.5 || v.Y < -2.5 {
				log.Panicf("you crashed! %f,%f", v.X, v.Y)
			}
			actors["player"].SetLinearVelocity(box2d.MakeB2Vec2(0, 0))
			log.Panicf("you landed safely!")
		}

		playerAngle = math.Mod(((actors["player"].GetAngle() - 0.5*math.Pi) * (180 / math.Pi)), 360)
		if playerAngle < 0 {
			playerAngle = 360 + playerAngle
		}
		position := actors["player"].GetPosition()
		playerX, playerY = position.X, position.Y
		playerAV = actors["player"].GetAngularVelocity()
		playerLV = actors["player"].GetLinearVelocity().Y
		app.QueueUpdateDraw(renderWorld)
	}
}

func main() {
	app = cview.NewApplication()

	setupInput()

	tv = cview.NewTextView()
	tv.SetDynamicColors(true)

	go runSimulation()

	app.SetRoot(tv, true)

	if err := app.Run(); err != nil {
		log.Fatal(err)
	}
}
