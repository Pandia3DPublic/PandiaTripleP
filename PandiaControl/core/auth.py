from flask import Blueprint, render_template, request, flash, redirect, url_for
from flask_login import login_user, login_required, current_user, logout_user
from .models import User
from werkzeug.security import check_password_hash

auth = Blueprint('auth', __name__)

@auth.route("/login", methods=["GET", "POST"])
def login():
    if request.method == "POST":
        email = request.form.get('n_EmailForm')
        password = request.form.get('n_PasswordForm')
        user = User.query.filter_by(email=email).first()
        if user:
            if check_password_hash(user.password, password):
                login_user(user, remember=False)
                return redirect(url_for('views.start'))
            else:
                flash('Incorrect Password', category='errorpw')
                error='id_PasswordForm'
                mailcache=email
        else:
            flash("Email does not exist", category='errormail')
            error='id_EmailForm'
            mailcache=email
    else: 
        error='0'
        mailcache=''
    return render_template("login.html", user=current_user, Error=error, Remember=mailcache)  # jinja2 just call it something


@auth.route("/logout", methods=["GET", "POST"])
@login_required
def logout():
    logout_user()
    return redirect(url_for('auth.login'))

