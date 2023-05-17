from pathlib import Path

from starlette.applications import Starlette
from starlette.routing import Mount, Route
from starlette.staticfiles import StaticFiles
from starlette.templating import Jinja2Templates

from robonotts.amy_cmd import amy_cmd_proxy

base_dir = directory = Path(__file__).parent

templates = Jinja2Templates(base_dir.joinpath("templates"))


async def homepage(request):
    return templates.TemplateResponse("index.html", {
        "request": request,
        "robot_name": "Happy",
        "rosbridge_host": "10.42.0.1",
        "rosbridge_port": 9091,
        })


app = Starlette(
    debug=True,
    routes=[
        Route("/", homepage),
        Route("/amy_cmd", amy_cmd_proxy),
        Mount(
            "/static",
            app=StaticFiles(directory=base_dir.joinpath("static")),
            name="static",
        ),
    ],
)
