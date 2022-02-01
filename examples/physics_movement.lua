--*******************************************************************************************
--
--   Physac - Physics movement
--
--   Copyright (c) 2016-2020 Victor Fisac (github: @victorfisac)
--
--   Adapted for yuema by megagrump@pm.me
--
--*******************************************************************************************/

require('lib.core.all')
local config, draw, window = yuema.config, yuema.draw, yuema.window
local time, keyboard = yuema.time, yuema.input.keyboard
local Font, Color = yuema.Font, yuema.Color
local Vector2 = require('lib.math.Vector2')
local physac = require('lib.physac')

local VELOCITY = 0.5
local screenWidth = 800
local screenHeight = 450

config.setFlags(config.MSAA_4X_HINT)
window.init(screenWidth, screenHeight, "[physac] - Body controller demo")
physac.init()

local logoX = screenWidth - Font.measureText("Physac", 30) - 10
local logoY = 15

do
	local floor = physac.createBodyRectangle(Vector2(screenWidth/2, screenHeight), screenWidth, 100, 10)
	local platformLeft = physac.createBodyRectangle(Vector2(screenWidth*0.25, screenHeight*0.6), screenWidth*0.25, 10, 10)
	local platformRight = physac.createBodyRectangle(Vector2(screenWidth*0.75, screenHeight*0.6), screenWidth*0.25, 10, 10)
	local wallLeft = physac.createBodyRectangle(Vector2(-5, screenHeight/2), 10, screenHeight, 10)
	local wallRight = physac.createBodyRectangle(Vector2(screenWidth + 5, screenHeight/2), 10, screenHeight, 10)

	floor.enabled = false
	platformLeft.enabled = false
	platformRight.enabled = false
	wallLeft.enabled = false
	wallRight.enabled = false
end

local body = physac.createBodyRectangle(Vector2(screenWidth/2, screenHeight/2), 50, 50, 1)
body.freezeOrient = true

time.setTargetFPS(60)

while not window.shouldClose() do
	if keyboard.isDown(keyboard.RIGHT) then
		body.velocity.x = VELOCITY
	elseif keyboard.isDown(keyboard.LEFT) then
		body.velocity.x = -VELOCITY
	end

	if keyboard.isDown(keyboard.UP) and body.isGrounded then
		body.velocity.y = -VELOCITY*4
	end

	draw.beginDrawing()
	draw.clearBackground(Color.BLACK)

	draw.fps(screenWidth - 90, screenHeight - 30)

	for i = 0, physac.getBodiesCount() - 1 do
		local body = physac.getBody(i)
		local vertexCount = physac.getShapeVerticesCount(i)
		for j = 0, vertexCount - 1 do
			vertexA = body:getShapeVertex(j)
			vertexB = body:getShapeVertex((j + 1) % vertexCount)
			draw.lineV(vertexA, vertexB, Color.GREEN)
		end
	end

	draw.text("Use 'ARROWS' to move player", 10, 10, 10, Color.WHITE)

	draw.text("Physac", logoX, logoY, 30, Color.WHITE)
	draw.text("Powered by", logoX + 50, logoY - 7, 10, Color.WHITE)

	draw.endDrawing()
end

physac.close()
window.close()
