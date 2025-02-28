from django import forms


class CommandForm(forms.Form):
    ip = forms.CharField(max_length=15)
    command = forms.CharField(max_length=100)
    args = forms.CharField(
        max_length=255,
        help_text="Аргументы через запятую (например: 255,0,100)"
    )
