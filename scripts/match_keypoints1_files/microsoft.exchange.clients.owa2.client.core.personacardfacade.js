﻿try { window.scriptsLoaded = window.scriptsLoaded || {}; window.scriptProcessStart = window.scriptProcessStart || {}; window.scriptProcessStart['microsoft.exchange.clients.owa2.client.core.personacardfacade.js'] = (new Date()).getTime();
Type.registerNamespace("_pcf");function PersonaCardFacadeComponent(){}PersonaCardFacadeComponent.prototype={$2FB:function(n,t,i){var f=this;n.$r(_pcf.$oN).$t(function(){return new _pcf.$oN(window.document.createElement("div"),n.$2(_b.$1U),n.$2(_y.$W),n.$2(_a.$1w))});var e=this;n.$r(_pcf.$oO).$2a(IPersonaCardViewModelFactory).$1I().$t(function(){return new _pcf.$oO(n.$2(_y.$W),n.$3O(IChatProviderFactory),n.$3O(IModernGroupCardViewModelFactory),n.$3O(IReadPersonaCardViewModelFactory),n.$3O(IGroupCardViewModelFactory),n.$3O(ILocationCardViewModelFactory),n.$3O(IComposePersonaCardViewModelFactory))});var r=this;n.$r(IModernGroupCardFacade).$1I().$t(function(){var t=n.$2(_pcf.$Ja);return t.$9(new _bc.$Eg,0)});var u=this;n.$r(_pcf.$Ja).$2a(IPersonaCardFacadeViewModelFactory).$1I().$t(function(){return new _pcf.$Ja(n.$2(_y.$W),n.$2(_a.$1d),n.$3O(IChatProviderFactory),n.$2(_pcf.$oO),n.$2(IMailComposeLauncher))})},$BjW:function(){return[]}};_pcf.$Ja=function(n,t,i,r,u){this.$I=n;this.$1G=t;this.$Jy=i;this.$10B=r;this.$13V=u};_pcf.$Ja.prototype={$I:null,$1G:null,$Jy:null,$13V:null,$10B:null,$9:function(n,t){return new _pcf.$IV(n,t,this.$10B,this.$1G,this.$I,this.$13V)},$wU:function(n,t,i){throw Error.notImplemented();},$1CI:function(n,t,i){var u=n;var r=this.$9(new _bc.$Eg,3);r.$CHI(u);this.$4rt();return r},$4rt:function(){if(this.$I.get_$U()==="Mouse"){var n=this;this.$Jy.$9(function(n){_g.$1H.$Yr.$4rt(n.$9())})}}};_pcf.$oO=function(n,t,i,r,u,f,e){this.$I=n;this.$Jy=t;this.$1MX=i;this.$6vq=r;this.$4oz=u;this.$6WA=f;this.$5rj=e};_pcf.$oO.prototype={$I:null,$Jy:null,$6vq:null,$4oz:null,$6WA:null,$5rj:null,$1MX:null,$AKf:function(n,t,i,r,u,f,e,o){switch(n.get_$24()){case 5:this.$AKj(n,t,u,f,e,o);break;case 2:this.$AKg(n,t,u,f,o);break;default:this.$AKl(n,t,u,f,o);break}},$AOx:function(n,t,i,r,u,f,e){switch(n.get_$24()){case 2:this.$AOy(n,t,r,u,f);break;case 4:this.$AP1(n,t,r,u,f);break;case 5:this.$AP2(n,t,r,u,f);break;default:this.$AP5(n,e,r,u,f);break}},$AP5:function(n,t,i,r,u){var f=this;this.$6vq.$9(function(f){f.$9(n,t,i,r,u)})},$AP2:function(n,t,i,r,u){if(this.$I.get_$U()!=="Mouse"){var e=this;this.$1MX.$9(function(f){var e=f.$7gO(n,i,t,null,r);e.$3e();u(e)})}else{var f=this;this.$Jy.$9(function(e){f.$1MX.$9(function(f){var o=f.$7gO(n,i,t,e.$9(),r);o.$3e();u(o)})})}},$AKj:function(n,t,i,r,u,f){var e=this;this.$1MX.$9(function(e){var o=e.$AKk(n,t,i,r,u);o.$3e();f(o)})},$AKl:function(n,t,i,r,u){var f=this;this.$5rj.$9(function(f){var e=f.$AKm(n,t,i,r);e.$3e();u(e)})},$AKg:function(n,t,i,r,u){var f=this;this.$4oz.$9(function(f){var e=f.$AKh(n,t,i,r);e.$3e();u(e)})},$AP1:function(n,t,i,r,u){var f=this;this.$6WA.$9(function(e){if(f.$I.get_$U()!=="Mouse"){var o=e.$7gG(n,i,t,null,r);o.$3e();u(o)}else f.$Jy.$9(function(f){var o=e.$7gG(n,i,t,f.$9(),r);o.$3e();u(o)})})},$AOy:function(n,t,i,r,u){var f=this;this.$4oz.$9(function(e){if(f.$I.get_$U()!=="Mouse"){var o=e.$7fp(n,i,t,null,r);o.$3e();u(o)}else f.$Jy.$9(function(f){var o=e.$7fp(n,i,t,f.$9(),r);o.$3e();u(o)})})}};_pcf.$IV=function(n,t,i,r,u,f){this.$$d_$7gU=Function.createDelegate(this,this.$7gU);this.$$d_$DpW=Function.createDelegate(this,this.$DpW);this.$$d_$Dul=Function.createDelegate(this,this.$Dul);this.$$d_$5DX=Function.createDelegate(this,this.$5DX);this.$$d_$DdX=Function.createDelegate(this,this.$DdX);this.$$d_$DUx=Function.createDelegate(this,this.$DUx);this.$$d_$Df6=Function.createDelegate(this,this.$Df6);this.$$d_$DP7=Function.createDelegate(this,this.$DP7);this.$$d_$4qb=Function.createDelegate(this,this.$4qb);this.$$d_$4X0=Function.createDelegate(this,this.$4X0);this.$$d_$ANx=Function.createDelegate(this,this.$ANx);this.$$d_$ANw=Function.createDelegate(this,this.$ANw);_pcf.$IV.initializeBase(this);this.$I=u;this.$1G=r;this.$xv=n;this.$3U1=t;this.$10B=i;this.$13V=f;this.$6sJ=new _y.$AU};_pcf.$IV.prototype={$I:null,$1G:null,$10B:null,$6sJ:null,$13V:null,$2G:null,$Jr:null,$T8:null,$2SS:null,$4wI:!1,$4u1:!1,$3U1:0,$xv:null,$c:null,$3In:!1,add_$9Et:function(n){this.$31("PersonaCreated",n)},remove_$9Et:function(n){this.$5d("PersonaCreated",n)},add_$7cZ:function(n){this.$31("ComposeCardChanged",n)},remove_$7cZ:function(n){this.$5d("ComposeCardChanged",n)},add_$7Wa:function(n){this.$31("BeforeComposeCardCreate",n)},remove_$7Wa:function(n){this.$5d("BeforeComposeCardCreate",n)},add_$2tG:function(n){this.$31("ReadCardChanged",n)},remove_$2tG:function(n){this.$5d("ReadCardChanged",n)},add_$4AD:function(n){this.$31("ReadCardChanging",n)},remove_$4AD:function(n){this.$5d("ReadCardChanging",n)},add_$Emb:function(n){this.$31("TodosButtonClicked",n)},get_$IQ:function(){this.$2G||(this.$2G=new _bc.$3e);return this.$2G},get_$DMq:function(){if(!this.$2SS){this.$2SS=new _js.$X;var n=!!this.$1G&&this.$1G.$2u("ModernGroups");var t=$C(_g.$O.get_$Q().get_$9K().get_$2k1())||_g.$O.get_$Q().get_$9K().get_$2k1();this.$2SS.$F(new _ff.$1l(new _C(this.$$d_$ANw,_pcf.$IV.$L),_nbs.NonBootStrings.l_CreateContactString_LowerCase_Text,n?_nbs.NonBootStrings.l_CreateContactDescription_Text:null));this.$I.get_$U()==="Mouse"&&this.$2SS.$F(new _ff.$1l(new _C(this.$$d_$ANx,_pcf.$IV.$L),n?_nbs.NonBootStrings.l_CreateContactListString_LowerCase_Text:_nbs.NonBootStrings.l_CreateGroupString_LowerCase_Text,n?_nbs.NonBootStrings.l_CreateContactListDescription_Text:null));n&&t&&this.$2SS.$F(new _ff.$1l(new _C(this.$$d_$4X0,_pcf.$IV.$L),_nbs.NonBootStrings.l_CreateGroupString_LowerCase_Text,_nbs.NonBootStrings.l_CreateGroupDescription_Text));this.$2SS.$F(new _ff.$1l(new _C(this.$$d_$4qb,_pcf.$IV.$L),_bcs.BootCommonStrings.l_ConfirmDialog_Cancel_Text,null))}return this.$2SS},set_$6R8:function(n){if(this.$4wI!==n){this.$4wI=n;this.$4("IsNewDialogShown")}return n},get_$2mB:function(){return this.$3In},set_$2mB:function(n){if(this.$3In===n)return n;this.$3In=n;this.$4("IsReadCardVisible");this.$Jr.set_$LR(this.$3In);return n},get_$Tk:function(){return this.$Jr},set_$Tk:function(n){if(this.$Jr===n)return n;this.$6S("ReadCardChanging");if(this.$Jr){this.set_$2mB(!1);this.$Jr.remove_$7TC(this.$$d_$DP7);this.$Jr.remove_$48s(this.$$d_$Df6);this.$Jr.get_$4AH().remove_$7lv(this.$$d_$DUx);this.$Jr.rpcl("IsVisible",this.$$d_$DdX);this.$Jr.dispose()}this.$Jr=n;if(this.$Jr){this.$Jr.apcl("IsVisible",this.$$d_$DdX);this.$Jr.add_$7TC(this.$$d_$DP7);this.$Jr.add_$48s(this.$$d_$Df6);this.$Jr.get_$4AH().add_$7lv(this.$$d_$DUx)}this.$6S("ReadCardChanged");this.$3h("ReadCard","HideConductor");return n},get_$1QK:function(){return this.$T8},set_$1QK:function(n){if(this.$T8===n)return n;this.$6S("ComposeCardChanging");if(this.$T8){this.$T8.remove_$7lw(this.$$d_$5DX);this.$T8.rpcl("IsVisible",this.$$d_$DdX);this.$T8.dispose()}this.$T8=n;if(this.$T8){this.$T8.apcl("IsVisible",this.$$d_$DdX);this.$T8.add_$7lw(this.$$d_$5DX)}this.$6S("ComposeCardChanged");this.$4("ComposeCard");return n},get_$9kz:function(){return new _C(this.$$d_$Dul,_a.$0.$QP)},$Xb:null,set_$10H:function(n){this.$Xb=n;return n},get_$2aN:function(){return this.$T8||this.$Jr},get_$CCX:function(){return _bc.$13n.isInstanceOfType(this.$I.$1Kr().get_$8p())},$1OK:function(n){if(this.$Jr&&_pc.$q.$31Q(this.$Jr.get_$3F(),n)){this.$7ky(!1);return}var i=_a.$6.acsa(_a.$0.$Fi,"CreatePersonaCardViewModel",!0);var t=this;this.$10B.$AOx(n,null,null,this.$xv,this,function(n){t.$9AW(n,!0,i)},this.$13V)},$33i:function(){this.set_$Tk(null)},$7gU:function(){this.$I.get_$U()==="Mouse"?this.set_$6R8(!0):this.$4Um(1)},$ANw:function(){this.$4qb();this.$4Um(1)},$ANx:function(){this.$4qb();this.$4Um(2)},$4X0:function(){this.$4qb();if(!this.$4u1){this.$4u1=!0;this.$4Um(5)}},$9ja:function(n,t){this.$5ss(n,2,t)},$5jc:function(n){_a.$6.$1f(_a.$0.$Fi,"PersonCard.AddToContacts");this.$5ss(n,3,new _rpc.$4O(null,3))},$AYR:function(n,t){this.$4bT(_pc.$q.$Ja(n),t)},$4bT:function(n,t){switch(n.get_$24()){case 1:_a.$6.stPT(_a.$0.$Hc,"EditContact",3);_a.$6.$1f(_a.$0.$Hc,"EditContact");break;case 2:_a.$6.stPT(_a.$0.$Hc,"EditGroup",3);_a.$6.$1f(_a.$0.$Hc,"EditGroup");break;case 5:_a.$6.stPT(_a.$0.$Hc,"EditModernGroup",3);_a.$6.$1f(_a.$0.$Hc,"EditModernGroup");break;default:break}this.$5ss(n,1,t)},$1C3:function(n){var u=n.get_$LR()&&(n.get_$5j()===2||n.get_$5j()===3||n.get_$3F().get_$Hd()===3);if(u){var t=null;var i=null;var r=n.get_$3F().get_$24();switch(n.get_$5j()){case 2:switch(r){case 1:t="NewContact";i="NewContact";break;case 2:t="NewGroup";i="NewGroup";break;case 5:t="NewModernGroup";i="NewModernGroup";break}break;case 1:switch(r){case 1:t="EditContact";i="EditContact";break;case 2:t="EditGroup";i="EditGroup";break}break;case 0:t="OpenContact";i="OpenPersonCard_"+this.$I.get_$U();break;case 3:t="PersonCard.AddToContacts";break}t&&(t==="OpenContact"&&this.$xv.$26r?_a.$6.$1F(null,t,null,null,"OBE"):_a.$6.$1F(null,t));i&&_a.$6.endPT(i)}},$CHI:function(n){this.$c=n;switch(this.$c.get_$5j()){case 3:this.$5jc(this.$c.get_$3F());break;case 1:this.$4bT(this.$c.get_$3F(),this.$Jr?this.$Jr.get_$4VV():new _rpc.$4O(null,1));break;case 2:var t=new _rpc.$4O(null,2);t.set_$3G(n.get_$3G());t.$y1=n.get_$5u5();this.$9ja(this.$c.get_$3F(),t);break;default:this.$1OK(this.$c.get_$3F());break}},$9y:function(){this.$T8&&this.$T8.get_$1Ti()?this.get_$IQ().$RH(2,this.$T8.get_$3F().get_$24()===5?[_nbs.NonBootStrings.l_ModernGroupCardDirtyPopoutWarning_Text]:[_nbs.NonBootStrings.l_PersonaCardDirtyPopoutWarning_Text],this.$$d_$DpW,!1,_bcs.BootCommonStrings.l_Yes_Text,_bcs.BootCommonStrings.l_No_Text,_nbs.NonBootStrings.l_PopoutCardConfirmationPromptTile_Text):this.$9Ef()},$2A:function(){this.set_$1QK(null);this.set_$Tk(null);_a.$1aV.prototype.$2A.call(this)},$9Ef:function(){var t=_a.$6.$1f(_a.$0.$Fi,"PersonCard.Popout",!0);if(this.$I.get_$U()!=="Mouse")throw Error.invalidOperation("Persona/Group Card can only be popped out in MUI");var n=this.$7f2();if(this.get_$2aN().get_$5j()){this.$T8.set_$1Ti(!1);this.$T8.get_$eN().$11()}this.get_$2aN().$1xi();this.$I.$VG(_pcf.$Ja,null,null,_bc.$2I.get_$Fi(),n,!1);_a.$6.$1F(t);_pc.$4r.$jq(32,8,this.get_$2aN().get_$3F().get_$24())},$DP7:function(){this.$5jc(this.$Jr.get_$3F())},$DUx:function(){this.$4bT(this.$Jr.get_$3F(),this.$Jr.get_$4VV())},$5ss:function(n,t,i){var r;switch(t){case 3:r="AddPersonaCardViewModel";break;case 2:r="CreatePersonaCardViewModel";break;case 1:r="EditPersonaCardViewModel";break;default:throw Error.invalidOperation(_js.$9.$5M("Cannot call CreateAndShowComposeCardViewModel with mode not in (Add,Create,Edit). Actual value = {0}",t));}var e=_a.$6.acsa(_a.$0.$Fi,r,!0);var u=this.$xv;if(t===1||t===3){u=new _bc.$Eg;u.$CC=this.$xv.$CC}i.set_$5j(t);this.$AQ("BeforeComposeCardCreate",i);var f=this;this.$10B.$AKf(n,t,null,null,u,this,i,function(n){f.$DSY(n,i,e)})},$5DX:function(n){var t=this.$T8.get_$5j();if(n)if(n.get_$24()===1){_a.$6.$1F(null,"SaveContact");_a.$6.endPT("SavePersona_"+this.$I.get_$U())}else if(n.get_$24()===2){_a.$6.$1F(null,"SaveGroup");_a.$6.endPT("SaveGroup_"+this.$I.get_$U())}if(this.$T8.get_$gw()){this.$T8.set_$1Ti(!1);this.$T8.get_$gw()()}(t===3||t===1)&&n&&this.$Jr&&this.$Jr.$Ddc(n);this.set_$1QK(null);t===2&&this.$AQ("PersonaCreated",n)},$DSY:function(n,t,i){_a.$6.acea(i);if(this.$c){this.$T8.$3ua(this.$c);this.$c=null}this.set_$1QK(n);this.$T8.$1TZ(t);if(this.$I.get_$Cp()||n.get_$5j()!==2){var r=_a.$6.acss(_a.$0.$Fi,"OwaShell.PopUp");this.$I.$Ax(this.$T8);_a.$6.aces(r)}else this.$6sJ.$NP(this.$T8);(this.$T8.get_$5j()===3||this.$T8.get_$5j()===1)&&this.$1C3(this.$T8);this.$4u1=!1},$7f2:function(){var n=new _rpc.$4O(this.get_$2aN(),this.get_$2aN().get_$5j());if(this.get_$2aN().get_$AE()){n.set_$4ze(this.get_$2aN().get_$AE().get_$4ze());n.set_$Xm(this.get_$2aN().get_$AE().get_$Xm())}if(this.$T8){n.set_$3G(this.$T8.get_$3G());n.set_$5u5(this.$T8.get_$5uA())}else n.set_$Gj(this.$Jr.get_$Gj());n.set_$1o5(!0);n.set_$ls(!1);return n},$DpW:function(n,t){n&&this.$9Ef()},$DdX:function(n,t){if(_rpc.$2F.isInstanceOfType(n)){var i=n;i.$Dt&&this.$1C3(i)}},$Df6:function(){this.$1C3(this.$Jr)},$9AW:function(n,t,i){_a.$6.acea(i);var r=!1;if(this.$c){r=!0;n.$3ua(this.$c);this.$c=null}this.set_$Tk(n);t&&this.$7ky(r)},$7ky:function(n){if(this.$3U1===3)if(n)this.set_$2mB(!0);else{this.$Jr.get_$4AH().set_$23G(!0);this.$Jr.$9Fk()}else if(this.$3U1)if(this.$3U1===1&&this.get_$CCX()){var i=_a.$6.acss(_a.$0.$Fi,"OwaShell.PopUp");this.$I.$lF(this.$Jr,1,!1);_a.$6.aces(i)}else{var t=_a.$6.acss(_a.$0.$Fi,"OwaShell.PopUp");this.$I.$Ax(this.$Jr);_a.$6.aces(t)}else this.set_$2mB(!0)},$Dul:function(){this.$6S("TodosButtonClicked")},$4qb:function(){this.set_$6R8(!1)},$4Um:function(n){var i="";n===1?i="NewContact":n===2?i="NewGroup":n===5&&(i="NewModernGroup");_a.$6.stPT(_a.$0.$Hc,i,3);_a.$6.$1f(_a.$0.$Hc,i);var t=_pc.$q.$5ul();t.set_$24(n);t.$Xb=this.$Xb;if(n===2){t.set_$2Z(new _g.$1K);t.get_$2Z().RoutingType="MAPIPDL"}this.$9ja(t,new _rpc.$4O(null,2))}};_pcf.$Mq=function(n){_pcf.$Mq.initializeBase(this,[n]);this.set_$6("PersonaCardFacadeView")};_pcf.$Mq.prototype={$2tH:null,get_$109:function(){return this.get_$k()},$4Q:function(){_js.$7.prototype.$4Q.call(this);this.$2tH=this.$u.$1N("PersonaCardView")}};_pcf.$oM=function(n){_pcf.$oM.initializeBase(this,[n])};_pcf.$oM.prototype={$2U3:null,$4Q:function(){_js.$7.prototype.$4Q.call(this);this.$2U3=this.$u.$1N("PersonaCardFacadeView")}};_pcf.$oN=function(n,t,i,r){_pcf.$oN.initializeBase(this,[n,t,i,r,"PersonaCardView"])};_pcf.$1NH=function(){};_pcf.$1NH.prototype={$4L:function(n,t){var i=n;switch(i){case 2:return"ReadGroupCardView";case 4:return"ReadLocationCardView";case 5:return"ReadModernGroupCardView";default:return"ReadPersonaCardView"}},$7dj:function(n,t){throw Error.notImplemented();}};_pcf._TI=function(){};_pcf._TI.$$cctor=function(){var t="PersonaCardFacadeView";new _A(t,function(){_pcf._TI.$1[t]===undefined&&(_pcf._TI.$1[t]=[[[-1,0,["MessageBoxViewModel"],[_pcf._TI.$Cps],null,"DataContext",null,_pcf._TI.$Y,1,null,null,null]],[[-1,4,null,null,null,"DialogTitle",null,_pcf._TI.$FS,0,null,null,null,_nbs.NonBootStrings.l_NewPeopleHubDialogTitleString_LowerCase_Text],[-1,0,["NewDialogButtons"],[_pcf._TI.$Cpt],null,"ButtonDataList",null,_pcf._TI.$FR,1,null,null,null],[-1,0,["IsNewDialogShown"],[_pcf._TI.$Cpq],_pcf._TI.$Cpu,"IsShown",_pcf._TI.$4w,_pcf._TI.$4f,2,null,null,!1]],[[-1,1,["PersonaCardFacadeViewModel","IsReadCardVisible"],[_pcf._TI.$Ctj,_pcf._TI.$Cpr],null,"IsHidden",null,_pcf._TI.$8,1,_pcf._TI.get_$K(),null,!0],[-1,0,["ReadCard"],[_pcf._TI.$1eg],null,"DataContext",null,_pcf._TI.$Y,1,null,null,null],[0,0,["TypeOfPersona"],[_pcf._TI.$DCe],null,"TemplateId",null,_pcf._TI.$Dg,1,_pcf._TI.get_$CtO(),null,null]]]);var n=_pcf._TI._hf.childNodes[0].cloneNode(!0);var i=new _pcf.$oN(n.children[2],_js.$2.Instance.$2(_b.$1U),_js.$2.Instance.$2(_y.$W),_js.$2.Instance.$2(_a.$1w));i.set_$g(-1);var u=new _b.$5L(n.children[1],_js.$2.Instance.$2(_y.$W),_js.$2.Instance.$2(_fc.$l),_js.$2.Instance.$2(_ff.$M),_js.$2.Instance.$2(_ff.$K));var r=new _js.$7(n.children[0]);r.set_$6("MessageBoxDefaultView");return new _B(n,[r,u,i]).R({PersonaCardView:i})},_pcf.$IV,_pcf.$Mq,!0,!1,!1,0,_pcf._TI.$1);var n="PersonaCardView";new _A(n,function(){_pcf._TI.$1[n]===undefined&&(_pcf._TI.$1[n]=[[[-1,0,["MessageBoxViewModel"],[_pcf._TI.$59P],null,"DataContext",null,_pcf._TI.$Y,1,null,null,null]]]);var i=_pcf._TI._hf.childNodes[1].cloneNode(!0);var t=new _js.$7(i.children[0]);t.set_$6("MessageBoxDefaultView");return new _B(i,[t])},_bc.$13n,_pcf.$oN,!0,!1,!1,0,_pcf._TI.$1)};_pcf._TI.$9V=function(){var n=window.document.createElement("DIV");n.innerHTML='<div> <div></div> <div autoid="_pcf_0"></div> <div></div> </div><div> <div></div> </div>';_js.$F.get_$9N().appendChild(n);return n};_pcf._TI.$Cps=function(n){return n.get_$IQ()};_pcf._TI.$Cpt=function(n){return n.get_$DMq()};_pcf._TI.$Cpq=function(n){return n.$4wI};_pcf._TI.$4w=function(n){return n.get_$1C()};_pcf._TI.$1eg=function(n){return n.$Jr};_pcf._TI.$DCe=function(n){return n.get_$5dC()};_pcf._TI.$Ctj=function(n){return n.get_$109()};_pcf._TI.$Cpr=function(n){return n.$3In};_pcf._TI.$59P=function(n){return n.get_$IQ()};_pcf._TI.$Y=function(n,t){n.set_$k(t)};_pcf._TI.$FS=function(n,t){n.set_$IC(t)};_pcf._TI.$FR=function(n,t){n.set_$I9(t)};_pcf._TI.$Cpu=function(n,t){n.set_$6R8(t)};_pcf._TI.$4f=function(n,t){n.set_$1C(t)};_pcf._TI.$Dg=function(n,t){n.set_$6(t)};_pcf._TI.$8=function(n,t){n.set_$17(t)};_pcf._TI.get_$CtO=function(){_pcf._TI.$4Ma||(_pcf._TI.$4Ma=new _pcf.$1NH);return _pcf._TI.$4Ma};_pcf._TI.get_$K=function(){_pcf._TI.$2M||(_pcf._TI.$2M=new _fc.$45);return _pcf._TI.$2M};PersonaCardFacadeComponent.registerClass("PersonaCardFacadeComponent",null,_a.$1XU);_pcf.$Ja.registerClass("_pcf.$Ja",null,_bc.$1JX,IPersonaCardFacadeViewModelFactory);_pcf.$oO.registerClass("_pcf.$oO",null,IPersonaCardViewModelFactory);_pcf.$IV.registerClass("_pcf.$IV",_a.$Em,_bc.$1WJ,IModernGroupCardFacade,_y.$1Y2);_pcf.$Mq.registerClass("_pcf.$Mq",_js.$7);_pcf.$oM.registerClass("_pcf.$oM",_js.$7);_pcf.$oN.registerClass("_pcf.$oN",_pc.$1ba);_pcf.$1NH.registerClass("_pcf.$1NH",null,_js.$1K7);_pcf._TI.registerClass("_pcf._TI");_pcf.$IV.$L=_a.$0.$Fi;_pcf._TI._hf=_pcf._TI.$9V();_pcf._TI.$4Ma=null;_pcf._TI.$2M=null;_pcf._TI.$1={};_pcf._TI.$$cctor();
window.scriptsLoaded['microsoft.exchange.clients.owa2.client.core.personacardfacade.js'] = 1; window.scriptProcessEnd = window.scriptProcessEnd || {}; window.scriptProcessEnd['microsoft.exchange.clients.owa2.client.core.personacardfacade.js'] = (new Date()).getTime(); } catch(e) { window.owaLastErrorReported = e; throw e; }
