from django.urls import path
from . import views

urlpatterns = [
    path('', views.send_command, name='send_command'),
]