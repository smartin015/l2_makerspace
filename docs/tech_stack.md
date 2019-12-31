# L2 Makerspace Tech Stack

## Requirements

Only requirement for makers: have a github account (or some git repository identity). 

Online and data

*   Have a full separate WAN & gigibit LAN for basement, with internet connectivity
*   MQTT available for quick pubsub stuff, interoperable with ROS2 telemetry
*   quick graphing/dashboarding (using SQL?)
*   Local Postgresql server with periodic backup to the cloud 
*   Each project is at core a github repo (and possibly blobstore? How do people handle large files in git?

Digital presence in the makerspace

*   Workspaces hosted on a main server - starts up a new session for each project (independent). A standard linux OS image is used.
*   Phone app allows selecting the project being worked on, also what's shown on individual screens (defaults to a workspace view).
*   Large screen displaying cost metrics, project statuses, other success indicators.
*   Several positional cameras (including a couple time lapse cameras) record when the space is active. Cameras can be moved without any manual calibration needed.
*   "Space is active" determined by enabled project app.
*   Operational dashboard shows what is and isn't working

Efficiency & reliability

*   Screens turn off when nobody in the space
*   After power cycling, should require 0 effort to get the space running again

App/server requirements

*   App must be multi-platform (web, ios, android)
*   Reports selected project and nearest iBeacon to an HTTP(s) endpoint on project/beacon delta.
*   Reports "exit" when far enough away from iBeacons, stops pinging the server.
*   REST style operations on project data, instructional steps, etc.
*   Server forwards anything interesting to MQTT for listening devices 
*   Can use [https://github.com/dtpnk/docker-vdi](https://github.com/dtpnk/docker-vdi) to create virtual desktops for users - customized with certain mounted file paths.
    *   Or maybe [https://isardvdi.readthedocs.io/en/latest/](https://isardvdi.readthedocs.io/en/latest/). There's a bunch of different docker-based VDI options.
*   Integration with dependencies
    *   [Todoist API](https://developer.todoist.com/sync/v8/)
    *   Need to reverse engineer instructables editing API (and Fiverr API?)
    *   

Bootstrapping

*   Should not be impossible to replicate this stack in other makerspaces. This requires a workflow/script that has both manual and automated steps to set up all the things. 

## User journeys:

New user to the space, wants to start on a project

1. Sign up for github account (if no account already)
2. Install android/ios app. In the app:
    *   Sign agreement to open-source all projects undertaken using L2 resources, affirm that all IP is self-owned
        *   As part of this, user signs in to github.
        *   Adds user to DB for tracking projects, adds as collaborator to various services (e.g. Todoist)
    *   Go through "new project" wizard
        *   Asks "what is the project? How does it use L2 making? What are the first steps?
        *   Forwards project form to application server, which
            *   Creates Todoist project and adds first steps there
            *   Creates Github repo
            *   Creates project container image, adds to DockerHub
            *   Creates scratch FS (with TTL) on server with checked out repository
3. Intro to the space
    *   Website & resources
    *   Cost tracker dashboard
    *   Electronics, including capabilities and the outsourcing process

Switching between projects

1. Open app, swipe in a direction to move to a different project
2. REST update sent to server changing selected project
3. Server forwards message to MQTT
4. All listening electronics update to the new project state
    1. Closest display to user switches to new project session

Server Bootstrapping from power-on

1. Start application server docker container (pulled from DockerHub?)
    1. Start connection to postgres DB and MQTT
    2. Discover devices in the space and react accordingly
        1. RPI VDI clients are forwarded various endpoints to transmit from, or a "sleep" command
        2. Cameras are connected and either start recording or sleep accordingly.
    3. Listen for instructions from project app instances

Existing user enters the space, opens project application

1. Server starts virtual desktop, awaits instructions on which screens to display it on
2. Server starts relevant cameras timelapsing/recording.

## Microprojects

*   Build a simple server with REST endpoints for
    *   Updating closets iBeacon to user
    *   Updating active project for user
    *   Adds/edits/archives maker project information to a postgres db
*   Build a cross-platform app that 
    *   sends the closest iBeacon information to a REST endpoint.
    *   submitting a project definition, instructions, and active project selection via REST
*   Build a raspberry pi image and accompanying docker VDI image that 
    *   Displays a cool logo when idle
    *   Can connect to a VDI image when given the address
    *   Connects (via websocket?) to a server binary and receives instructions on which image to display
    *   Can access the GPU?
*   Build a python module that syncs postgres project instances to Todoist and syncs Todoist tasks to Postgres
*   Build a python module that creates github repositories and can commit files when given a user auth token
*   Build a dashboard of running containers vs what's expected to run (encoded as postgres table?). This should be usable as a touchscreen UI.
*   Build a camera recording system container which stores video on a large hard drive on the server.
