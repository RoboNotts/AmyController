from pathlib import Path

from starlette.applications import Starlette
from starlette.routing import Mount, Route
from starlette.staticfiles import StaticFiles
from starlette.templating import Jinja2Templates

base_dir = directory = Path(__file__).parent

templates = Jinja2Templates(base_dir.joinpath("templates"))


async def homepage(request):
    return templates.TemplateResponse("index.html", {
        "request": request,
        "robot_name": "Happy",
        })


app = Starlette(
    debug=True,
    routes=[
        Route("/", homepage),
        Mount(
            "/static",
            app=StaticFiles(directory=base_dir.joinpath("static")),
            name="static",
        ),
    ],
)
