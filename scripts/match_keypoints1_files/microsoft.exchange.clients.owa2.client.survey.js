﻿try { window.scriptsLoaded = window.scriptsLoaded || {}; window.scriptProcessStart = window.scriptProcessStart || {}; window.scriptProcessStart['microsoft.exchange.clients.owa2.client.survey.js'] = (new Date()).getTime();
Type.registerNamespace("_sy");function SurveyComponent(){}SurveyComponent.prototype={$2FB:function(n,t,i){var u=this;n.$r(_sy.$xY).$2a(ISurveyFactory).$1I().$t(function(){return new _sy.$xY});var r=this;n.$r(_sy.$18b).$2a(ISurveyDialogViewModelFactory).$1I().$t(function(){return new _sy.$18b})},$BjW:function(){return[]}};_sy.$1BP=function(n,t,i,r,u,f,e){this.$$d_$3RI=Function.createDelegate(this,this.$3RI);_sy.$1BP.initializeBase(this,[n,t,i,r,u,f,e]);this.$8U=new _C(this.$$d_$3RI,f);this.$PR.set_$7DY(!1);this.$6Uu=_ff.$8c.$3Ty(-this.$5w7,0)};_sy.$1BP.prototype={$5w7:10,$6Uu:0,$8U:null,$2Ec:function(){if(this.$7aB()){this.$NQ();return}if(!_sy.$7q.$23X){this.$6n.get_$3hY()||this.$6n.set_$3hY(_a.$8.get_utcNow().$4N(this.$6Uu));this.$AF.get_$oq()&&_a.$g.get_$CE().toLowerCase()==="en-us"&&this.$I.get_$U()==="Mouse"&&(!this.$PR.get_$5Xm()||this.$PR.get_$5Xm()())&&this.$6n.get_$3hY().$4N(this.$5w7).$3s(_a.$8.get_utcNow())<0&&this.$NQ()}},$NQ:function(){this.set_$Bx3(this.$8U);this.set_$Bx2(_nbs.NonBootStrings.l_SurveyDialogButtonCancel_Text);this.$4("HandleLaterOrDismissCommand");this.$4("HandleLaterOrDismissButtonText");this.set_$3u(this.$PR.get_$3u());this.set_$1C(!0);_sy.$7q.$23X=!0},$Dhw:function(){if(this.$9wT()){var n=null;!this.get_$6Sx()&&this.$PR.get_$5FC()&&(n=this.$PR.get_$5FC()(this.$1Re,this.$1XH,this.$1lK));this.$2Y.$4Ib(this.$PR.get_$121(),_y.$3c.toString(this.$1Re),_y.$3c.toString(this.$1XH),this.$1lK,!0,!!this.$6n.get_$1BM(),!1,this.$1OU,n);this.$6n.set_$3hY(_a.$8.get_utcNow());_h.$1z.$Kn(this.$6n.$Hl,OwaApplication.$2B);this.$N.$1o(_a.$CT,new _a.$CT);this.$5a.$11();_sy.$7q.$23X=!1}},$3RI:function(){this.$6n.set_$3hY(_a.$8.get_utcNow());_h.$1z.$Kn(this.$6n.$Hl,OwaApplication.$2B);this.$2Y.$4Ib(this.$PR.get_$121(),_y.$3c.toString(this.$1Re),_y.$3c.toString(this.$1XH),this.$1lK,!1,!0,!1,this.$1OU,null);this.$5a.$11();_sy.$7q.$23X=!1}};_sy.$xY=function(){};_sy.$xY.prototype={$9:function(n,t,i,r,u,f,e,o,s,h,c,l,a,v){return new _sy.$18a(n,t,i,r,u,f,e,o,s,h,c,l,a,v)},$7hM:function(n){var t=null;n===6&&(t=this.$ANP());return t},$ANP:function(){return new _sy.$18a(6,_nbs.NonBootStrings.l_MailSatisfactionSurveyDialogTitle_Text,null,_nbs.NonBootStrings.l_MailSatisfactionSurveyDialogStatement_Text,_nbs.NonBootStrings.l_SurveyDialog_High_Text,_nbs.NonBootStrings.l_SurveyDialog_Low_Text,5,_nbs.NonBootStrings.l_MailSatisfactionSurveyDialogQuestion1_Text,_nbs.NonBootStrings.l_MailSatisfactionSurveyDialogQuestion2_Text,_nbs.NonBootStrings.l_MailSatisfactionSurveyDialogQuestion1_Required_Text,_nbs.NonBootStrings.l_MailSatisfactionSurveyDialogQuestion2_Required_Text,null,null,_nbs.NonBootStrings.l_MailSatisfactionSurveyDialog_CommentsLabel_Text)}};_sy.$18b=function(){};_sy.$18b.prototype={$9:function(n,t,i,r,u,f,e){return t.get_$121()===5?new _sy.$1BP(n,t,i,r,u,f,e):new _sy.$7q(n,t,i,r,u,f,e)},$AQ9:function(n,t,i,r,u,f,e){var s=new _sy.$xY;var o=s.$7hM(n);return o?new _sy.$7q(t,o,i,r,u,f,e):null}};_sy.$18a=function(n,t,i,r,u,f,e,o,s,h,c,l,a,v){this.$5vS=n;this.$6pU=a;this.$7E2=l;this.$7w=t;this.$4v=i;this.$5ZY=r;this.$396=this.$5OD=e;this.$62o=o;this.$62p=h;if(s){this.$71N=s;this.$71O=c;this.$7EE=!0}this.$4Gj=!0;this.$4VC=v?v:_nbs.NonBootStrings.l_SurveyDialogAdditionalComments_Text;if(this.$396===5||this.$396===3){this.$4eB=this.$3W1=_nbs.NonBootStrings.l_SurveyDialog_One_Text;this.$62s=this.$71Q=_nbs.NonBootStrings.l_SurveyDialog_Two_Text;this.$62t=this.$4CG=_nbs.NonBootStrings.l_SurveyDialog_Three_Text;this.$62q=this.$71P=_nbs.NonBootStrings.l_SurveyDialog_Four_Text;this.$4eA=this.$3W0=_nbs.NonBootStrings.l_SurveyDialog_Five_Text;this.$7Oy=String.format(_nbs.NonBootStrings.l_SurveyDialog_Equals_Text,_nbs.NonBootStrings.l_SurveyDialog_One_Text,f);this.$5mr=String.format(_nbs.NonBootStrings.l_SurveyDialog_Equals_Text,_nbs.NonBootStrings.l_SurveyDialog_Five_Text,u)}else if(this.$396===2){this.$4eB=this.$3W1=f;this.$4eA=this.$3W0=u}else _a.$4.$7IX("surveyScaleToUse","The survey scale is not supported.")};_sy.$18a.prototype={$62o:null,$71N:null,$62p:null,$71O:null,$5vS:0,$7w:null,$4v:null,$5ZY:null,$6pU:null,$7E2:null,$5mr:null,$7Oy:null,$7EE:!1,$4Gj:!1,$4VC:null,$396:0,$5OD:0,$4eB:null,$62s:null,$62t:null,$62q:null,$4eA:null,$3W1:null,$71Q:null,$4CG:null,$71P:null,$3W0:null,$5yL:!1,get_$121:function(){return this.$5vS},get_$3u:function(){return this.$7w},get_$6Z:function(){return this.$4v},get_$Ei8:function(){return this.$5ZY},get_$Aph:function(){return this.$62o},get_$E5v:function(){return this.$71N},get_$Api:function(){return this.$62p},get_$E5w:function(){return this.$71O},get_$A8E:function(){return this.$5mr},get_$EzP:function(){return this.$7Oy},get_$62r:function(){return this.$396},get_$5OE:function(){return this.$5OD},set_$5OE:function(n){this.$5OD=n;return n},get_$9kl:function(){return this.$7EE},get_$7DY:function(){return this.$4Gj},set_$7DY:function(n){this.$4Gj!==n&&(this.$4Gj=n);return n},get_$CBV:function(){return _js.$9.$o(this.$5ZY)?!0:!1},get_$CBU:function(){return _js.$9.$o(this.$4v)?!0:!1},get_$5FC:function(){return this.$6pU},get_$5Xm:function(){return this.$7E2},get_$Apk:function(){return this.$4eB},get_$Apm:function(){return this.$62s},get_$Apn:function(){return this.$62t},get_$Apl:function(){return this.$62q},get_$Apj:function(){return this.$4eA},get_$9Q2:function(){return this.$3W1},set_$9Q2:function(n){this.$3W1!==n&&(this.$3W1=n);return n},get_$E5y:function(){return this.$71Q},get_$9Q3:function(){return this.$4CG},set_$9Q3:function(n){this.$4CG!==n&&(this.$4CG=n);return n},get_$E5x:function(){return this.$71P},get_$9Q1:function(){return this.$3W0},set_$9Q1:function(n){this.$3W0!==n&&(this.$3W0=n);return n},get_$7l0:function(){return this.$5yL},set_$7l0:function(n){this.$5yL=n;return n}};_sy.$7q=function(n,t,i,r,u,f,e){this.$$d_$DUQ=Function.createDelegate(this,this.$DUQ);this.$$d_$DY4=Function.createDelegate(this,this.$DY4);this.$$d_$Dhw=Function.createDelegate(this,this.$Dhw);_sy.$7q.initializeBase(this);this.$I=n;this.$AF=u;this.$N=i;this.$6n=r;this.set_$FA(this);this.set_$1oM(!0);this.$PR=t;this.$1OU=r.get_$37N();this.$2D=f;this.$72p=new _C(this.$$d_$Dhw,f);this.$2Y=e;if(!this.$6n.get_$2P5()){this.$6n.set_$2P5(_a.$8.get_utcNow().$4N(-_sy.$7q.$4Yj));_h.$1z.$Kn(this.$6n.$Hl,OwaApplication.$2B)}this.set_$3jN("SurveyDialogView")};_sy.$7q.prototype={$I:null,$2Y:null,$72p:null,$3FB:null,$2D:null,$PR:null,$1Re:0,$1XH:0,$1lK:"",$6n:null,$N:null,$AF:null,$1OU:!1,$3FA:null,get_$DsM:function(){return String.format(_nbs.NonBootStrings.l_SurveyDialogPrivacyNotice_Text,"Survey/"+_a.$g.get_$3dr()+"_SurveyPrivacyStatement.htm")},set_$Bx3:function(n){this.$3FB!==n&&(this.$3FB=n);return n},set_$Bx2:function(n){this.$3FA!==n&&(this.$3FA=n);return n},get_$Eao:function(){return this.$PR.get_$62r()===3||this.$PR.get_$5OE()===3&&this.$PR.get_$62r()!==5&&this.$PR.get_$5OE()!==5},get_$6Sx:function(){return!this.$1Re&&!this.$1XH},$2Ec:function(){if(this.$7aB()){this.$NQ();return}this.$ADK();if(this.$PR.get_$121()===1&&this.$6n.get_$6Q0()){var n=!1;if((this.$6n.get_$1xm()&1)!=1){var t;(t=this.$6n).set_$1xm(t.get_$1xm()|1);n=!0}if((this.$6n.get_$1BM()&1)==1){this.$6n.set_$1BM(0);n=!0}n&&_h.$1z.$tf(this.$6n.$Hl,OwaApplication.$2B);return}this.$AF.get_$oq()&&_a.$g.get_$CE().toLowerCase()==="en-us"&&this.$I.get_$U()==="Mouse"&&!this.$6n.get_$37N()&&(this.$6n.get_$1xm()&this.$PR.get_$121())!==this.$PR.get_$121()&&(this.$6n.get_$1BM()?(this.$6n.get_$1BM()&this.$PR.get_$121())===this.$PR.get_$121()&&this.$6n.get_$2P5().$2cc(_sy.$7q.$8MR).$3s(_a.$8.get_utcNow())<0&&this.$NQ():this.$6n.get_$2P5().$4N(_sy.$7q.$4Yj).$3s(_a.$8.get_utcNow())<0&&(!this.$PR.get_$5Xm()||this.$PR.get_$5Xm()())&&this.$NQ())},$NQ:function(){if(this.$6n.get_$1BM()){this.$3FB=new _C(this.$$d_$DUQ,this.$2D);this.$3FA=_u.Strings.l_Dismiss_Text}else{this.$3FB=new _C(this.$$d_$DY4,this.$2D);this.$3FA=_nbs.NonBootStrings.l_SurveyDialogButtonCancel_Text}this.$4("HandleLaterOrDismissCommand");this.$4("HandleLaterOrDismissButtonText");this.set_$3u(this.$PR.get_$3u());this.set_$1C(!0);_sy.$7q.$23X=!0},$Dhw:function(){if(this.$9wT()){var t=null;!this.get_$6Sx()&&this.$PR.get_$5FC()&&(t=this.$PR.get_$5FC()(this.$1Re,this.$1XH,this.$1lK));this.$2Y.$4Ib(this.$PR.get_$121(),_y.$3c.toString(this.$1Re),_y.$3c.toString(this.$1XH),this.$1lK,!0,!!this.$6n.get_$1BM(),!1,this.$1OU,t);this.$6n.set_$2P5(_a.$8.get_utcNow());this.$6n.set_$1BM(0);var n;(n=this.$6n).set_$1xm(n.get_$1xm()|this.$PR.get_$121());this.$6n.set_$37N(this.$1OU);_h.$1z.$Kn(this.$6n.$Hl,OwaApplication.$2B);this.$N.$1o(_a.$CT,new _a.$CT);this.$5a.$11();_sy.$7q.$23X=!1}},$DY4:function(){this.$6n.set_$2P5(_a.$8.get_utcNow());this.$6n.set_$1BM(this.$PR.get_$121());this.$6n.set_$37N(this.$1OU);_h.$1z.$Kn(this.$6n.$Hl,OwaApplication.$2B);this.$2Y.$4Ib(this.$PR.get_$121(),_y.$3c.toString(this.$1Re),_y.$3c.toString(this.$1XH),this.$1lK,!1,!0,!1,this.$1OU,null);this.$5a.$11();_sy.$7q.$23X=!1},$DUQ:function(){this.$6n.set_$2P5(_a.$8.get_utcNow());this.$6n.set_$1BM(0);var t;(t=this.$6n).set_$1xm(t.get_$1xm()|this.$PR.get_$121());var n;(n=this.$6n).set_$5y0(n.get_$5y0()|this.$PR.get_$121());this.$6n.set_$37N(this.$1OU);_h.$1z.$Kn(this.$6n.$Hl,OwaApplication.$2B);this.$2Y.$4Ib(this.$PR.get_$121(),_y.$3c.toString(this.$1Re),_y.$3c.toString(this.$1XH),this.$1lK,!1,!1,!0,this.$1OU,null);this.$5a.$11();_sy.$7q.$23X=!1},$9wT:function(){var t=!0;var n=new(_ff.$18.$$(String));if(!this.get_$6Sx()||!this.$1OU){if(!this.$1Re){n.$F(this.$PR.get_$Api());t=!1}if(this.$PR.get_$9kl()&&!this.$1XH){n.$F(this.$PR.get_$E5w());t=!1}n.get_$A()>0&&_a.$3t.$RH(n.$3T(),0)}return t},$ADK:function(){if(!(this.$6n.get_$1BM()===this.$PR.get_$121())&&!!this.$6n.get_$1BM()&&this.$6n.get_$2P5().$4N(_sy.$7q.$4Yj).$3s(_a.$8.get_utcNow())<0){this.$6n.set_$1BM(0);_h.$1z.$Kn(this.$6n.$Hl,OwaApplication.$2B)}},$7aB:function(){var n=_a.$2B.$4of(window.location.href,"survey");return this.$PR.get_$7l0()||n&&n==="1"?!0:!1}};_sy._TI=function(){};_sy._TI.$$cctor=function(){var f="SurveyDialogView._tid1";new _A(f,function(){_sy._TI.$1[f]===undefined&&(_sy._TI.$1[f]=[[[-1,0,["Survey","FirstQuestion"],[_sy._TI.$VC,_sy._TI.$CMf],null,"Text",null,_sy._TI.$H,1,null,null,null],[-1,5,["Survey","FirstQuestionScale"],[_sy._TI.$VC,_sy._TI.$52W],null,"_sy_k",null,null,0,new _js.$x([_sy._TI.get_$6V(),_sy._TI.get_$K()],[5,null]),null,null]],[[-1,0,["Survey","BestOptionText"],[_sy._TI.$VC,_sy._TI.$8dU],null,"Text",null,_sy._TI.$H,1,null,null,null]],[[-1,0,["Survey","WorstOptionText"],[_sy._TI.$VC,_sy._TI.$8dV],null,"Text",null,_sy._TI.$H,1,null,null,null]],[[-1,4,null,null,null,"Data",null,_sy._TI.$8L,0,null,null,null,1],[-1,0,["Survey","FirstQuestionFirstRadioButtonText"],[_sy._TI.$VC,_sy._TI.$CMh],null,"Text",null,_sy._TI.$8M,1,null,null,null],[-1,5,["ShouldUseThreeLevelScaleSpacing"],[_sy._TI.$3P4],null,"_sy_9",null,null,0,null,null,null]],[[-1,4,null,null,null,"Data",null,_sy._TI.$8L,0,null,null,null,2],[-1,0,["Survey","FirstQuestionSecondRadioButtonText"],[_sy._TI.$VC,_sy._TI.$CMj],null,"Text",null,_sy._TI.$8M,1,null,null,null],[-1,0,["Survey","FirstQuestionScale"],[_sy._TI.$VC,_sy._TI.$52W],null,"IsHidden",null,_sy._TI.$8,1,new _js.$x([_sy._TI.get_$6V(),_sy._TI.get_$K()],[5,null]),null,!0]],[[-1,4,null,null,null,"Data",null,_sy._TI.$8L,0,null,null,null,3],[-1,0,["Survey","FirstQuestionThirdRadioButtonText"],[_sy._TI.$VC,_sy._TI.$CMk],null,"Text",null,_sy._TI.$8M,1,null,null,null],[-1,0,["Survey","FirstQuestionScale"],[_sy._TI.$VC,_sy._TI.$52W],null,"IsHidden",null,_sy._TI.$8,1,_sy._TI.get_$6V(),2,!0],[-1,5,["ShouldUseThreeLevelScaleSpacing"],[_sy._TI.$3P4],null,"_sy_9",null,null,0,null,null,null]],[[-1,4,null,null,null,"Data",null,_sy._TI.$8L,0,null,null,null,4],[-1,0,["Survey","FirstQuestionFourthRadioButtonText"],[_sy._TI.$VC,_sy._TI.$CMi],null,"Text",null,_sy._TI.$8M,1,null,null,null],[-1,0,["Survey","FirstQuestionScale"],[_sy._TI.$VC,_sy._TI.$52W],null,"IsHidden",null,_sy._TI.$8,1,new _js.$x([_sy._TI.get_$6V(),_sy._TI.get_$K()],[5,null]),null,!0]],[[-1,4,null,null,null,"Data",null,_sy._TI.$8L,0,null,null,null,5],[-1,0,["Survey","FirstQuestionFifthRadioButtonText"],[_sy._TI.$VC,_sy._TI.$CMg],null,"Text",null,_sy._TI.$8M,1,null,null,null],[-1,5,["ShouldUseThreeLevelScaleSpacing"],[_sy._TI.$3P4],null,"_sy_9",null,null,0,null,null,null]]]);var n=_sy._TI._hf.childNodes[0].cloneNode(!0);var u=new _fce.$2w(n.children[5]);u.set_$6("SurveyDialogView.CustomRadioButton");u.set_$g(0);var e=new _fce.$2w(n.children[4]);e.set_$6("SurveyDialogView.CustomRadioButton");e.set_$g(0);var r=new _fce.$2w(n.children[3]);r.set_$6("SurveyDialogView.CustomRadioButton");r.set_$g(0);var t=new _fce.$2w(n.children[2]);t.set_$6("SurveyDialogView.CustomRadioButton");t.set_$g(0);var i=new _fce.$2w(n.children[1]);i.set_$6("SurveyDialogView.CustomRadioButton");i.set_$g(0);var h=new _fc.$A(_B.$3(n,[0,2]));h.set_$g(0);var s=new _fc.$A(_B.$3(n,[0,1]));s.set_$g(0);var o=new _fc.$A(_B.$3(n,[0,0]));o.set_$g(0);return new _B(n,[o,s,h,i,t,r,e,u])},_sy.$7q,_fce.$3W,!1,!0,!1,0,_sy._TI.$1);var u="SurveyDialogView._tid2";new _A(u,function(){_sy._TI.$1[u]===undefined&&(_sy._TI.$1[u]=[[[-1,0,["Survey","SecondQuestion"],[_sy._TI.$VC,_sy._TI.$CMm],null,"Text",null,_sy._TI.$H,1,null,null,null],[-1,5,["Survey","SecondQuestionScale"],[_sy._TI.$VC,_sy._TI.$52X],null,"_sy_k",null,null,0,new _js.$x([_sy._TI.get_$6V(),_sy._TI.get_$K()],[5,null]),null,null]],[[-1,0,["Survey","BestOptionText"],[_sy._TI.$VC,_sy._TI.$8dU],null,"Text",null,_sy._TI.$H,1,null,null,null]],[[-1,0,["Survey","WorstOptionText"],[_sy._TI.$VC,_sy._TI.$8dV],null,"Text",null,_sy._TI.$H,1,null,null,null]],[[-1,4,null,null,null,"Data",null,_sy._TI.$8L,0,null,null,null,1],[-1,0,["Survey","SecondQuestionFirstRadioButtonText"],[_sy._TI.$VC,_sy._TI.$CMo],null,"Text",null,_sy._TI.$8M,1,null,null,null],[-1,5,["ShouldUseThreeLevelScaleSpacing"],[_sy._TI.$3P4],null,"_sy_9",null,null,0,null,null,null]],[[-1,4,null,null,null,"Data",null,_sy._TI.$8L,0,null,null,null,2],[-1,0,["Survey","SecondQuestionSecondRadioButtonText"],[_sy._TI.$VC,_sy._TI.$CMq],null,"Text",null,_sy._TI.$8M,1,null,null,null],[-1,0,["Survey","SecondQuestionScale"],[_sy._TI.$VC,_sy._TI.$52X],null,"IsHidden",null,_sy._TI.$8,1,new _js.$x([_sy._TI.get_$6V(),_sy._TI.get_$K()],[5,null]),null,!0]],[[-1,4,null,null,null,"Data",null,_sy._TI.$8L,0,null,null,null,3],[-1,0,["Survey","SecondQuestionThirdRadioButtonText"],[_sy._TI.$VC,_sy._TI.$CMr],null,"Text",null,_sy._TI.$8M,1,null,null,null],[-1,0,["Survey","SecondQuestionScale"],[_sy._TI.$VC,_sy._TI.$52X],null,"IsHidden",null,_sy._TI.$8,1,_sy._TI.get_$6V(),2,!0],[-1,5,["ShouldUseThreeLevelScaleSpacing"],[_sy._TI.$3P4],null,"_sy_9",null,null,0,null,null,null]],[[-1,4,null,null,null,"Data",null,_sy._TI.$8L,0,null,null,null,4],[-1,0,["Survey","SecondQuestionFourthRadioButtonText"],[_sy._TI.$VC,_sy._TI.$CMp],null,"Text",null,_sy._TI.$8M,1,null,null,null],[-1,0,["Survey","SecondQuestionScale"],[_sy._TI.$VC,_sy._TI.$52X],null,"IsHidden",null,_sy._TI.$8,1,new _js.$x([_sy._TI.get_$6V(),_sy._TI.get_$K()],[5,null]),null,!0]],[[-1,4,null,null,null,"Data",null,_sy._TI.$8L,0,null,null,null,5],[-1,0,["Survey","SecondQuestionFifthRadioButtonText"],[_sy._TI.$VC,_sy._TI.$CMn],null,"Text",null,_sy._TI.$8M,1,null,null,null],[-1,5,["ShouldUseThreeLevelScaleSpacing"],[_sy._TI.$3P4],null,"_sy_9",null,null,0,null,null,null]]]);var n=_sy._TI._hf.childNodes[0].cloneNode(!0);var f=new _fce.$2w(n.children[5]);f.set_$6("SurveyDialogView.CustomRadioButton");f.set_$g(0);var e=new _fce.$2w(n.children[4]);e.set_$6("SurveyDialogView.CustomRadioButton");e.set_$g(0);var r=new _fce.$2w(n.children[3]);r.set_$6("SurveyDialogView.CustomRadioButton");r.set_$g(0);var t=new _fce.$2w(n.children[2]);t.set_$6("SurveyDialogView.CustomRadioButton");t.set_$g(0);var i=new _fce.$2w(n.children[1]);i.set_$6("SurveyDialogView.CustomRadioButton");i.set_$g(0);var h=new _fc.$A(_B.$3(n,[0,2]));h.set_$g(0);var s=new _fc.$A(_B.$3(n,[0,1]));s.set_$g(0);var o=new _fc.$A(_B.$3(n,[0,0]));o.set_$g(0);return new _B(n,[o,s,h,i,t,r,e,f])},_sy.$7q,_fce.$3W,!1,!0,!1,0,_sy._TI.$1);var o="SurveyDialogView._tid3";new _A(o,function(){_sy._TI.$1[o]===undefined&&(_sy._TI.$1[o]=[[[-1,4,null,null,null,"Text",null,_sy._TI.$H,0,null,null,null,_nbs.NonBootStrings.l_SurveyDialogButtonOK_Text]]]);var n=_sy._TI._hf.childNodes[1].cloneNode(!0);var t=new _fc.$A(n.children[0]);return new _B(n,[t])},_sy.$7q,_fc.$H,!1,!0,!1,0,_sy._TI.$1);var e="SurveyDialogView._tid4";new _A(e,function(){_sy._TI.$1[e]===undefined&&(_sy._TI.$1[e]=[[[-1,0,["HandleLaterOrDismissButtonText"],[_sy._TI.$DEd],null,"Text",null,_sy._TI.$H,1,null,null,null]]]);var n=_sy._TI._hf.childNodes[2].cloneNode(!0);var t=new _fc.$A(n.children[0]);return new _B(n,[t])},_sy.$7q,_fc.$H,!1,!0,!1,0,_sy._TI.$1);var t="SurveyDialogView.CustomRadioButton";new _A(t,function(){_sy._TI.$1[t]===undefined&&(_sy._TI.$1[t]=[null,[[-1,1,["Text"],[_sy._TI.$8ze],null,"Text",null,_sy._TI.$H,1,null,null,null]]]);var n=_sy._TI._hf.childNodes[3].cloneNode(!0);var r=new _fc.$A(n.children[1]);var i=new _js.$F(n.children[0]);return new _B(n,[i,r]).R({RadioButton:i,RadioButtonText:r})},Object,_fce.$2w,!1,!1,!1,0,_sy._TI.$1);var n="SurveyDialogView.CustomButton._tid5";new _A(n,function(){_sy._TI.$1[n]===undefined&&(_sy._TI.$1[n]=[[[-1,0,["Label"],[_sy._TI.$59m],null,"Text",null,_sy._TI.$H,1,null,null,null]]]);var i=_sy._TI._hf.childNodes[4].cloneNode(!0);var t=new _fc.$A(i.children[0]);return new _B(i,[t]).R({Label:t})},_ff.$1l,_fc.$H,!1,!0,!1,0,_sy._TI.$1);var r="SurveyDialogView.CustomButton";new _A(r,function(){_sy._TI.$1[r]===undefined&&(_sy._TI.$1[r]=[[[-1,0,["ClickCommand"],[_sy._TI.$59k],null,"ClickCommand",null,_sy._TI.$Z,1,null,null,null],[-1,0,["IsHidden"],[_sy._TI.$59l],null,"IsHidden",null,_sy._TI.$8,1,null,null,!0]]]);var t=_sy._TI._hf.childNodes[5].cloneNode(!0);var n=new _fc.$H(t.children[0]);n.set_$4j(1);n.set_$6("SurveyDialogView.CustomButton._tid5");return new _B(t,[n])},_ff.$1l,_js.$7,!1,!1,!1,0,_sy._TI.$1);var i="SurveyDialogView";new _A(i,function(){_sy._TI.$1[i]===undefined&&(_sy._TI.$1[i]=[[[-1,0,["Survey","Message"],[_sy._TI.$VC,_sy._TI.$CMl],null,"Text",null,_sy._TI.$H,1,null,null,null],[-1,0,["Survey","IsNullOrEmptyMessage"],[_sy._TI.$VC,_sy._TI.$DEZ],null,"IsHidden",null,_sy._TI.$8,1,null,null,!0]],[[-1,0,["Survey","Statement"],[_sy._TI.$VC,_sy._TI.$CMu],null,"Text",null,_sy._TI.$H,1,null,null,null],[-1,0,["Survey","IsNullOrEmptyStatement"],[_sy._TI.$VC,_sy._TI.$DEa],null,"IsHidden",null,_sy._TI.$8,1,null,null,!0]],[[-1,0,["FirstQuestionAnswer"],[_sy._TI.$DEc],_sy._TI.$DEk,"SelectedOptionData",_sy._TI.$LU,_sy._TI.$LV,2,null,null,null]],[[-1,0,["SecondQuestionAnswer"],[_sy._TI.$DEg],_sy._TI.$DEl,"SelectedOptionData",_sy._TI.$LU,_sy._TI.$LV,2,null,null,null],[-1,0,["Survey","ShowSecondQuestion"],[_sy._TI.$VC,_sy._TI.$CMt],null,"IsHidden",null,_sy._TI.$8,1,_sy._TI.get_$K(),null,!0]],[[-1,0,["Survey","CommentsLabel"],[_sy._TI.$VC,_sy._TI.$DEY],null,"Text",null,_sy._TI.$H,1,null,null,null]],[[-1,0,["FeedbackComments"],[_sy._TI.$DEb],_sy._TI.$DEj,"Text",_sy._TI.$5K,_sy._TI.$5G,2,null,null,null]],[[-1,4,null,null,null,"Text",null,_sy._TI.$1V,0,null,null,null,_nbs.NonBootStrings.l_SurveyDialogDontShowDialog_Text],[-1,0,["SurveyDontShowAgainStatus"],[_sy._TI.$DEi],_sy._TI.$DEm,"IsChecked",_sy._TI.$61,_sy._TI.$4p,2,null,null,!1],[-1,0,["Survey","ShowDontShowAgainCheckBox"],[_sy._TI.$VC,_sy._TI.$CMs],null,"IsHidden",null,_sy._TI.$8,1,_sy._TI.get_$K(),null,!0]],[[-1,0,["SendFeedbackCommand"],[_sy._TI.$DEh],null,"ClickCommand",null,_sy._TI.$Z,1,null,null,null]],[[-1,0,["HandleLaterOrDismissCommand"],[_sy._TI.$DEe],null,"ClickCommand",null,_sy._TI.$Z,1,null,null,null]],[[-1,4,null,null,null,"Text",null,_sy._TI.$H,0,null,null,null,_nbs.NonBootStrings.l_SurveyDialogThanksMessage_Text]],[[-1,4,null,null,null,"Text",null,_sy._TI.$H,0,null,null,null,_nbs.NonBootStrings.l_SurveyDialogNote_Text]],[[-1,0,["PrivacyNotice"],[_sy._TI.$DEf],null,"Html",null,_sy._TI.$Ix,1,null,null,null]],[[-1,4,null,null,null,"Text",null,_sy._TI.$H,0,null,null,null,_nbs.NonBootStrings.l_SurveyDialogCopyright_Text]]]);var n=_sy._TI._hf.childNodes[6].cloneNode(!0);var l=new _fc.$A(n.children[9]);l.set_$g(0);var c=new _b.$4B(n.children[8]);c.set_$g(0);var h=new _fc.$A(n.children[7]);h.set_$g(0);var y=new _fc.$A(n.children[6]);y.set_$g(0);var v=new _fc.$H(_B.$3(n,[5,1]));v.set_$6("SurveyDialogView._tid4");var a=new _fc.$H(_B.$3(n,[5,0]));a.set_$6("SurveyDialogView._tid3");var s=new _fc.$2p(n.children[4],_js.$2.Instance.$2(_ff.$C));s.set_$g(0);var t=new _fce.$AN(_B.$3(n,[3,1]));t.set_$g3("_sy_n");t.set_$g(0);var r=new _fc.$A(_B.$3(n,[3,0]));r.set_$g(0);var f=new _fce.$3W(_B.$3(n,[2,1]));f.set_$6("SurveyDialogView._tid2");var u=new _fce.$3W(_B.$3(n,[2,0]));u.set_$6("SurveyDialogView._tid1");var e=new _fc.$A(n.children[1]);e.set_$g(0);var o=new _fc.$A(n.children[0]);o.set_$g(0);return new _B(n,[o,e,u,f,r,t,s,a,v,y,h,c,l])},_sy.$7q,_js.$7,!0,!1,!1,0,_sy._TI.$1)};_sy._TI.$9V=function(){var n=window.document.createElement("DIV");n.innerHTML="<div> <div class='_sy_o'> <span class='_sy_2 ms-font-color-black _sy_j'></span> <span class='_sy_7'></span> <span class='_sy_7'></span> </div> <label class='_sy_8'></label> <label class='_sy_8'></label> <label class='_sy_8'></label> <label class='_sy_8'></label> <label class='_sy_8'></label> </div><div> <span class='customButtonText'></span> </div><div> <span class='customButtonText'></span>'\n            </div><div> <input type=\"radio\" class='_sy_a'/> <span class='_sy_b'></span> </div><div> <span></span> </div><div> <button type='button' class='_sy_m ms-bg-color-neutralLighter'></button> </div><div> <span class='_sy_2 ms-font-color-black _sy_3'></span> <span class='_sy_2 ms-font-color-black _sy_4'></span> <div class='_sy_5'> <div class='_sy_6'></div> <div class='_sy_6'></div> </div> <div class='_sy_c'> <span class='_sy_2 ms-font-color-black _sy_d'></span> <textarea class='_sy_e ms-border-color-neutralTertiary'></textarea> </div> <button type='button' class='_sy_i'></button> <div class='_sy_l'> <button type='button' class='_sy_m ms-bg-color-themePrimary ms-font-color-white'></button> <button type='button' class='_sy_m ms-bg-color-neutralLighter'></button> </div> <span class='_sy_2 ms-font-color-black _sy_f'></span> <span class='_sy_g'></span> <span class='_sy_g'></span> <span class='_sy_g _sy_h'></span>   </div>";_js.$F.get_$9N().appendChild(n);return n};_sy._TI.$VC=function(n){return n.$PR};_sy._TI.$CMf=function(n){return n.get_$Aph()};_sy._TI.$52W=function(n){return n.get_$62r()};_sy._TI.$8dU=function(n){return n.get_$A8E()};_sy._TI.$8dV=function(n){return n.get_$EzP()};_sy._TI.$CMh=function(n){return n.get_$Apk()};_sy._TI.$3P4=function(n){return n.get_$Eao()};_sy._TI.$CMj=function(n){return n.get_$Apm()};_sy._TI.$CMk=function(n){return n.get_$Apn()};_sy._TI.$CMi=function(n){return n.get_$Apl()};_sy._TI.$CMg=function(n){return n.get_$Apj()};_sy._TI.$CMm=function(n){return n.get_$E5v()};_sy._TI.$52X=function(n){return n.get_$5OE()};_sy._TI.$CMo=function(n){return n.get_$9Q2()};_sy._TI.$CMq=function(n){return n.get_$E5y()};_sy._TI.$CMr=function(n){return n.get_$9Q3()};_sy._TI.$CMp=function(n){return n.get_$E5x()};_sy._TI.$CMn=function(n){return n.get_$9Q1()};_sy._TI.$DEd=function(n){return n.$3FA};_sy._TI.$8ze=function(n){return n.$3S};_sy._TI.$59m=function(n){return n.$NF};_sy._TI.$59k=function(n){return n.$Lp};_sy._TI.$59l=function(n){return n.$88};_sy._TI.$CMl=function(n){return n.get_$6Z()};_sy._TI.$DEZ=function(n){return n.get_$CBU()};_sy._TI.$CMu=function(n){return n.get_$Ei8()};_sy._TI.$DEa=function(n){return n.get_$CBV()};_sy._TI.$DEc=function(n){return n.$1Re};_sy._TI.$LU=function(n){return n.get_$cQ()};_sy._TI.$DEg=function(n){return n.$1XH};_sy._TI.$CMt=function(n){return n.get_$9kl()};_sy._TI.$DEY=function(n){return n.$4VC};_sy._TI.$DEb=function(n){return n.$1lK};_sy._TI.$5K=function(n){return n.get_$1K()};_sy._TI.$DEi=function(n){return n.$1OU};_sy._TI.$61=function(n){return n.$HS};_sy._TI.$CMs=function(n){return n.get_$7DY()};_sy._TI.$DEh=function(n){return n.$72p};_sy._TI.$DEe=function(n){return n.$3FB};_sy._TI.$DEf=function(n){return n.get_$DsM()};_sy._TI.$H=function(n,t){n.set_$1K(t)};_sy._TI.$8L=function(n,t){n.$1e=t};_sy._TI.$8M=function(n,t){n.set_$1K(t)};_sy._TI.$8=function(n,t){n.set_$17(t)};_sy._TI.$Z=function(n,t){n.set_$4k(t)};_sy._TI.$DEk=function(n,t){n.$1Re=t};_sy._TI.$LV=function(n,t){n.set_$cQ(t)};_sy._TI.$DEl=function(n,t){n.$1XH=t};_sy._TI.$DEj=function(n,t){n.$1lK=t};_sy._TI.$5G=function(n,t){n.set_$1K(t)};_sy._TI.$DEm=function(n,t){n.$1OU=t};_sy._TI.$4p=function(n,t){n.set_$8x(t)};_sy._TI.$1V=function(n,t){n.set_$1K(t)};_sy._TI.$Ix=function(n,t){n.set_$SU(t)};_sy._TI.get_$6V=function(){_sy._TI.$DL||(_sy._TI.$DL=new _fc.$ES);return _sy._TI.$DL};_sy._TI.get_$K=function(){_sy._TI.$2M||(_sy._TI.$2M=new _fc.$45);return _sy._TI.$2M};SurveyComponent.registerClass("SurveyComponent",null,_a.$1XU);_sy.$7q.registerClass("_sy.$7q",_bc.$DH,_y.$1X1);_sy.$1BP.registerClass("_sy.$1BP",_sy.$7q);_sy.$xY.registerClass("_sy.$xY",null,ISurveyFactory);_sy.$18b.registerClass("_sy.$18b",null,ISurveyDialogViewModelFactory);_sy.$18a.registerClass("_sy.$18a",null,ISurvey);_sy._TI.registerClass("_sy._TI");_sy.$7q.$23X=!1;_sy.$7q.$4Yj=21;_sy.$7q.$8MR=172;_sy._TI._hf=_sy._TI.$9V();_sy._TI.$DL=null;_sy._TI.$2M=null;_sy._TI.$1={};_sy._TI.$$cctor();
window.scriptsLoaded['microsoft.exchange.clients.owa2.client.survey.js'] = 1; window.scriptProcessEnd = window.scriptProcessEnd || {}; window.scriptProcessEnd['microsoft.exchange.clients.owa2.client.survey.js'] = (new Date()).getTime(); } catch(e) { window.owaLastErrorReported = e; throw e; }
