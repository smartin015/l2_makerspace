# Level 2 Makerspace

## Mission

Disrupt maker culture by enabling faster, easier, and more effective solutions to more complex and intelligent problems through the use of increasing computation, technical expertise, and investment in open discovery.


## Background

### Stages

The core aspects of making can be divided into three stages: identifying a **problem** or project, finding/proposing a **solution** for the project, and **fabrication** or execution of the solution. This parallels the scientific method (hypothesis, testing, and evaluation) and is often cyclical, even tiered - solutions to the major problem may encounter sub-problems in fabrication that require their own solutions, etc.

Making is also an intersection of engineering and experimentation. This strongly parallels "infrastructure" development for experimental research (e.g. as is done with [Deepmind](http://deepmind.com)).


### Progression

Making can be improved at each of these stages by 1) reducing the time to the next stage, and b) reducing the attention and effort required to move to the next stage. Machine/software automation improves both attention and execution speed, but is hard to achieve as a first resort. 

We take a series of smaller steps to reach full autonomy, using a human approach to fill the gaps and steadily decreasing the gap until full automation can be achieved:



*   **Manual**: 100% attention of maker required
*   **Outsourced**: 20% attention of maker, with unpaid/paid remote assistance
*   **Assisted**: 20% attention, remaining aided by software/machine
*   **Automated**: Almost 0% attention, with nearly all aspects handled by software/machine 


### Levels

Earlier stages in the making process require more creativity and thoughtfulness than later stages, making them harder to automate. To simplify discussion, we can define "levels" similar to those used when describing [vehicular autonomy](https://www.google.com/search?q=vehicle+autonomy+levels&rlz=1CAKMJF_enUS867US867&oq=vehicle+autonomy+levels&aqs=chrome..69i57j69i60l3j69i61j69i60.1999j0j1&sourceid=chrome&ie=UTF-8).


<table>
  <tr>
   <td><strong>Level</strong>
   </td>
   <td><strong>Problem </strong>
   </td>
   <td><strong>Solution</strong>
   </td>
   <td><strong>Fabrication</strong>
   </td>
  </tr>
  <tr>
   <td>0
   </td>
   <td>manual
   </td>
   <td>manual
   </td>
   <td>manual
   </td>
  </tr>
  <tr>
   <td>1
   </td>
   <td>manual
   </td>
   <td>manual
   </td>
   <td>manual/assisted
   </td>
  </tr>
  <tr>
   <td>1E
   </td>
   <td>manual
   </td>
   <td>outsourced
   </td>
   <td>manual/assisted
   </td>
  </tr>
  <tr>
   <td>2
   </td>
   <td>manual
   </td>
   <td>assisted
   </td>
   <td>outsourced/assisted
   </td>
  </tr>
  <tr>
   <td>3
   </td>
   <td>outsourced
   </td>
   <td>assisted 
   </td>
   <td>assisted/automated
   </td>
  </tr>
  <tr>
   <td>4
   </td>
   <td>assisted
   </td>
   <td>automated
   </td>
   <td>automated
   </td>
  </tr>
  <tr>
   <td>5
   </td>
   <td>automated
   </td>
   <td>automated
   </td>
   <td>automated
   </td>
  </tr>
</table>




*   Most makerspaces fall into either Level 0 or Level 1 - basic wood shops, metal shops, etc. fall into Level 0, while makerspaces that leverage 3D printing, CNC machining, etc. are considered Level 1.
*   Today's Level 2 spaces are typically research spaces, companies, or institutions - and the capabilities of these spaces are often highly targeted to solving a small class of problems.
*   The workshops in the Ironman movies describe a futuristic Level 4 space.
*   At level 5, a maker could give a statement of intent (e.g. a "mission") and have an a-la-carte menu of problems and their solutions presented automatically. The maker can explore and tweak various parameters and then fabricate the solution with trivial effort.


### Barriers & Opportunity



*   Technical expertise is needed to harness today's L2 tools, and to know what tools even exist / whether integration is possible
    *   E.g. finite element analysis when designing structures, or generative design, or even just regular CAD tools like Fusion 360
*   Expensive compute / machinery is often foregone, with L0/L1 tools considered sufficient for most makers (which isn't wrong, but is certainly limiting)
    *   E.g. protohaven using donated and underpowered "library" computers
*   Most makerspaces target self-sufficiency / profit, externalizing cost onto makers.
*   TPUs and specialized computing for machine learning is rapidly improving.
*   Sites like [fiverr](fiverr.com) allow for inexpensive, contracted solutions but are typically not utilized by makers today.
*   Making has entrenched a particular view of DIY, where making something with your own hands is valued more than the thing being made. 


## Objective



*   By the end of 2020, understand how to make a feasible L2 space and should be making progress on a 1E space.
*   By the end of 2021, achieve L2 for a makerspace in Pittsburgh.


## Requirements



*   The design of the space, its progress, and all projects within it must be **open source and publicized**. 
*   **Low-effort maintenance** is required for all aspects of the L2 space
*   Creations must be **repeatable**, including the creation of an L2 space
*   Newcomers should be able to **use the space in an hour or less**


## Execution

Executing the mission is a multi-year undertaking, with substantial monetary and time investment and distributed across many people.


### Project Management

Projects will be managed with [Todoist](todoist.com), with sub-parts broken up into manageable pieces and delegated to experts. A visible display will be set up in the makerspace and its items distributed across various roles/makers. 

When a user wants to create a new project, there should be an easy workflow to automatically:



*   Create a Todoist project and add some "setup" steps to the project for the maker or contracted admins to do:
    *   complete design document
    *   complete concept video
    *   spec budget 
*   Create an empty design document (and link it into the project)
*   Create a (network, backed up) storage directory for videos/pictures taken as part of the project

This project's mission is about meta-making: the act of making in order to improve the act of making. This must itself be a project, but should be tracked alongside all other projects as above (and prioritized as below).


### Administration



*   To ensure success and tracking of the project, project management will be done by a paid assistant (e-assistant, part-time?) who will also handle hiring of temporary workers to "fill in automation gaps".
*   To ensure the makerspace is maintained, operational, and utilized, a salaried "maker in residence" will be hired to steward the space, execute on projects, and invent clever solutions for meta-making ($800/wk?).
*   To ensure documentation happens, video/instruction editing will be contracted (see "documentation" below)
*   Outreach (both to satisfy maker problems and to seek new makers) will also be contracted.

All of these administrative roles must also be considered as opportunities for automation (e.g. automatic pipelining of contracted work given a "spend" budget)


### Budgeting

Budget/Spend must be tracked and visible within the makerspace, with a rolling 7-day limit to encourage wise (and complete) use of resources and categorization. Expenses must be itemized for tax purposes (use quickbooks for all this? Alternatively [this basic sheet](https://docs.google.com/spreadsheets/d/1BBIxhmSKlSGzjuNiQGqYvpwJ9ztJQF3Wk-ob_apegaI/edit#gid=0)). 

Can open a new checking account and have money auto-deposited in it.

Budget for 2020 will be used to explore concepts and prepare for the push for L2 in 2021.

Budget for 2021 will be roughly $440 accrued daily, before deductions for salary etc. This can be applied both to services and to materials for projects.


### Documentation & Open Source

Initially, manual work must be undertaken for:



*   docwriting and manuals/instruction for makerspace tooling
*   Youtube videos and other promotional materials showing off the space
*   Instructables posts and other writeups of project current state

Moving forward, projects are undertaken to lessen this manual time and effort. For example:



*   Button activated time lapse and "vlog" cameras for easier video capture
*   Scripts to transcribe / summarize video, or to convert to e.g. instructables format and automatically post sections of the initial design document


### Profit Model

Ideas executed in the makerspace may have business potential. However:



*   IP ownership transfer should be compelling, but not compulsive. 
*   The maker should by default own any physical/fabricated thing, and it shouldn't permanently live in the makerspace unless it facilitates the goals of meta-making.

We may also sell design services (e.g. mocap) to companies, or "in-source the out-source" to solve the needs of folks not in Pittsburgh. However, this should not be our primary motivator.

We could additionally bring in instructors or teachers to describe L2 making techniques. This could be free for locals and sold online to cover any excessive cost.


### Capabilities / Tools

See also [L2 Making Tech Stack](https://docs.google.com/document/d/1h5DuauvpZ7f_S83kQJ1KfR9Ppj57YCTjlgOnyPH5PmM/edit?usp=sharing)



*   Cutting, bending, and working metal 
    *   Start with CNC plasma, metal brake, TIG welder, [5-axis CNC machine](https://pocketnc.com/collections/machines/products/pocket-nc-v2?variant=11607895998511), Induction welder
*   Digital <-> physical 
    *   Motion capture
    *   High quality A/V recording
    *   Vesa mount touch monitor screens/consoles (e.g. [link](https://amzn.to/34mWqWg))
    *   Maker-kiva (donut-shaped table w/ screens n'at, w/ mobile monitor on rails)
    *   Multiple embedded screens, connects to VNC for particular person when they're nearby
    *   Directional / digitally controlled lights to indicate what actions to perform for manual, not-yet-automated tools and processes (for faster onboarding of users)
*   Composites for fancy and sleek shapes
    *   Carbon fiber, reel, 2-part resin and layup tools (heated wire, chicken wire?)
    *   UV cure powder coat (coats carbon fiber!) and UV lamps
*   Simulation & design
    *   [Simplified design system](https://hackaday.com/2019/10/18/replacing-the-3d-printer-and-router-a-tool-for-manufacturing-human-scale-forms/), potentially automatable
    *   Generative CAD
    *   Pregen primitives e.g. [https://www.templatemaker.nl/en/](https://www.templatemaker.nl/en/) 
    *   Evolutionary algorithms for making more optimal problems
    *   Gazebo or other physics simulation 
    *   Auto generation of compliant mechanisms
    *   [ML based PCB routing](https://hackaday.com/2019/12/01/deeppcb-routes-your-kicad-pcbs/)
*   Fixturing & cleanup
    *   Helping hands++ (coffee grounds jamming arms, with manipulators?)
    *   [Loc-line](https://www.loc-line.com/) ventilation endpoints
    *   Cardboard & packaged plastics recycling


## Education & Inspiration

Even at L2, much of the identification of problems and solutions would need to be the role of the maker. As a result, we should have ready materials for self-study of various topics to boost ceativity, e.g: 



*   Robotics
*   Animatronics
*   Sensors & signals

Additionally we could offer a feed of Inspiration - at least a list of links to standout makers' playlists like Colin Furze and/or projects such as Deeplocal's exhibits/installations. This could evolve into a "maker marquis" similar to a TV at reception that shows off projects being undetaken in the space as well as fun published videos of other goings-on in the maker community.


## Application & prioritization

Due to the smaller size of the space, external users must apply - describing their goals, usage patterns, and skills, as well as describing what they want to do which isn't satisfied by an L1 makerspace. Projects are similarly "application" based and are ranked qualitatively, with some preference given to the below:



1. Meta-making (make it easier/quicker to make)
    *   Scripting/automating common maker actions, improved pipelining/project management, etc
2. Recycling/reuse for making
    *   E.g. Mass conversion of cardboard/plastic/etc into building materials for projects
    *   Making it easier to recycle
3. Time-saving inventions
    *   JIT versions of batch appliances
        *   dishwasher+dryer in cupboard
        *   clothes hamper washing machine (or even light-soil instant washing/steaming machine)
        *   Easy-cleanup "drinkbot"
4. Home improvement
    *   E.g. high-quality furniture that doubles as interactive/art installations, or serves additional functional purpose.
5. Experiences
    *   Animatronics, aesthetic/themed rooms with interactive behavior, etc.
    *   New ways of blending virtual and real?


## Success Metrics



*   2+ large published projects (video + instructions)
*   3+ small published projects (video + instructions)
*   Budget fully utilized
*   At least 1 email sent to external makerspaces / labs / makers per week


## Standards



*   Docker containers for all software projects
*   All solutions must work with Linux? 
*   Use SI units for dimensions?
*   


## 2020 H1



*   See [Todoist Project](https://todoist.com/app#project%2F_1573253044751%2Ffull)


## 2020 H2



*   Start advertising / drumming up interest
*   HIre an e-assistant to begin administrative work.
    *   Connect with other makerspaces and start a collaborative relationship
    *   Start talking with various groups in the area to discuss possible involvement... maybe a few months in advance (start in July/August?)
        *   Need folks invested in building out the new space. Also to break it into specific projects.
*   Space upgrades
    *   Start ordering tools before arriving back in PIT
    *   Plan out system requirements for multiple high-quality time lapse cameras, etc.
*   Continue building a queue of projects / interested folks for a good start in 2021.

At the end of 2020, we should be set to have a Level 1E makerspace.


## 2021 H1

Start identifying automatic solutions for outsourced bits of projects


## 2021 H2

By the end of the year, achieve Level 2.


## Advertising

For maker in residence:

*   Full time salaried position ($20/hr) building the next generation of makerspaces. 
    *   Try out new, futuristic making tools and techniques
    *   Help automate the slow parts of making
    *   Includes unlimited (daytime) access to the makerspace, with potential budget for self-designed, self-owned projects. 
*   Some makerspace experience (e.g. TechShop, Protohaven) desirable, but not required
*   Must be familiar with typical machine shop tools and safety
*   Must have basic knowledge of networking and computer programming

For project makers:

*   Join our Level 2 makerspace
*   Get a grant for building your next project
*   Fully own your ideas from start to finish
*   Free access to next gen tooling and consumables (e.g. carbon fiber and generative design)
*   Be part of a global effort to take making to the next level
*   Apply today! (link to application, possibly also QR for printed flyers)

Project maker application (fill out once per project; link to main site of L2 space)

*   Name, contact details etc
*   Please briefly describe your background (and give any examples of prior completed projects)
*   Project name
*   What is the goal of your project? What's it do/solve?
*   Why is this project a good fit for the L2 makerspace?
*   <checkbox to confirm that IP will be owned by the user, but the project itself *must* be fully open-source. Link to a "development flow" page>

Web site (squarespace hosted)

*   Homepage
    *   Include concept art, plus ways to apply for maker in residence, projects
    *   Summarize high level mission and the major points (radically transparent, open source etc)
    *   Sitemap to other pages
*   Mission
    *   An HTML version of this doc
*   Projects
    *   Fancy listing of user projects and their statuses/sources
        *   Read-only view of Todoist for L2
        *   Current budget / spend and items/services purchased, aggregated at top
    *   Separate tabs: 
        *   L2 sub-projects / bootstrapping for makerspaces
        *   User projects
*   Apply
    *   Links to application forms
    *   Also link to contact address (for inquiries/visits/tours)
