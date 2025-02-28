from django.shortcuts import render, redirect
from django.http import JsonResponse
import requests
import json
from .forms import CommandForm


def send_command(request):
    if request.method == 'POST':
        form = CommandForm(request.POST)
        if form.is_valid():
            ip = form.cleaned_data['ip']
            cmd = {
                "cmd": form.cleaned_data['command'],
                "args": [arg.strip() for arg in form.cleaned_data['args'].split(',')]
            }

            try:
                # Отправка JSON на IP устройства
                response = requests.post(
                    f"http://{ip}/command",
                    json=cmd,
                    timeout=5
                )
                return JsonResponse({"status": "success", "response": response.text})
            except Exception as e:
                return JsonResponse({"status": "error", "message": str(e)})
    else:
        form = CommandForm()

    return render(request, 'sender/send_command.html', {'form': form})